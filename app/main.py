from __future__ import annotations

import asyncio
import json
import math
import os
from contextlib import asynccontextmanager
from dataclasses import replace
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from pyproj import CRS
from pyproj.exceptions import CRSError

from app.api_models import Calibration2DInput, ConfigInput, PortInput
from app.config import AppConfig, ImuConfig, LeverConfig, NtripConfig, load_config, save_config
from app.runtime import Runtime

CONFIG_PATH = Path(os.getenv("VIRTUAL_ARM_CONFIG", "config.toml"))
FRONTEND_PATH = Path(__file__).resolve().parents[1] / "frontend" / "index.html"
runtime = Runtime(load_config(CONFIG_PATH))
config_write_lock = asyncio.Lock()

_FIELD_NAMES = {
    "host": "Servidor NTRIP",
    "port": "Puerto",
    "mountpoint": "Mountpoint",
    "username": "Usuario",
    "password": "Contraseña NTRIP",
    "projected_crs": "Sistema de coordenadas",
    "forward_m": "Adelante desde ANT1",
    "left_m": "Izquierda desde ANT1",
    "down_m": "Abajo desde ANT1",
    "easting_m": "Este del punto",
    "northing_m": "Norte del punto",
}


@asynccontextmanager
async def lifespan(_: FastAPI):
    await runtime.start()
    try:
        yield
    finally:
        await runtime.stop()


app = FastAPI(
    title="",
    docs_url=None,
    redoc_url=None,
    openapi_url=None,
    lifespan=lifespan,
)


@app.exception_handler(RequestValidationError)
async def readable_validation_error(_: Request, exc: RequestValidationError) -> JSONResponse:
    """Pydantic devuelve `detail` como lista de objetos; la pantalla lo mostraba
    como `[object Object]`. Se aplana a una frase que nombra el campo."""
    parts: list[str] = []
    for error in exc.errors():
        location = [str(item) for item in error.get("loc", []) if item != "body"]
        field = _FIELD_NAMES.get(location[-1] if location else "", ".".join(location))
        message = error.get("msg", "valor inválido")
        parts.append(f"{field}: {message}" if field else message)
    return JSONResponse(status_code=422, content={"detail": " · ".join(parts) or "Datos inválidos"})


def public_config(config: AppConfig) -> dict:
    return {
        "ntrip": {
            "host": config.ntrip.host,
            "port": config.ntrip.port,
            "mountpoint": config.ntrip.mountpoint,
            "username": config.ntrip.username,
            "has_password": config.ntrip.has_password(),
        },
        "coordinates": {"projected_crs": config.projected_crs},
        "lever": {
            "forward_m": config.lever.forward_m,
            "left_m": config.lever.left_m,
            "down_m": config.lever.down_m,
        },
        "calculation": {"use_imu": config.use_imu, "use_pitch": config.use_pitch},
    }


@app.get("/")
async def index() -> FileResponse:
    # Sin esto, tras actualizar el servicio la tablet sigue mostrando la interfaz
    # vieja desde su caché y el operador no entiende por qué nada cambió.
    return FileResponse(FRONTEND_PATH, headers={"Cache-Control": "no-store"})


@app.get("/api/config")
async def get_config() -> dict:
    return public_config(runtime.config)


def merged_config(current: AppConfig, incoming: ConfigInput) -> AppConfig:
    return replace(
        current,
        projected_crs=incoming.coordinates.projected_crs,
        lever=LeverConfig(
            forward_m=incoming.lever.forward_m,
            left_m=incoming.lever.left_m,
            down_m=incoming.lever.down_m,
        ),
        use_imu=incoming.calculation.use_imu,
        use_pitch=incoming.calculation.use_pitch,
        ntrip=NtripConfig(
            host=incoming.ntrip.host,
            port=incoming.ntrip.port,
            mountpoint=incoming.ntrip.mountpoint,
            username=incoming.ntrip.username,
            password_file=current.ntrip.password_file,
            gga_interval_s=current.ntrip.gga_interval_s,
        ),
    )


def validate_projected_crs(code: str) -> None:
    """Rechaza los CRS geográficos.

    PROJ acepta `WGS84` o `EPSG:4326` sin protestar, y entonces el Este y el
    Norte de la pantalla pasan a ser grados sin que nada lo delate.
    """
    crs = CRS.from_user_input(code)
    if crs.is_geographic or not crs.is_projected:
        raise ValueError(
            f"«{code}» es un sistema geográfico en grados, no una proyección en metros. "
            "Use el código proyectado del topógrafo, por ejemplo EPSG:32717"
        )
    units = {axis.unit_name for axis in crs.axis_info}
    if not units <= {"metre", "meter"}:
        raise ValueError(f"«{code}» no está en metros ({', '.join(sorted(units))})")


@app.put("/api/config")
async def put_config(incoming: ConfigInput) -> dict:
    async with config_write_lock:
        updated = merged_config(runtime.config, incoming)
        try:
            # Valida el CRS antes de tocar archivos o la sesión activa.
            from app.geometry import Geodesy

            validate_projected_crs(updated.projected_crs)
            Geodesy(updated.projected_crs)
        except (CRSError, ValueError) as exc:
            raise HTTPException(status_code=400, detail=f"{exc}") from exc
        save_config(updated, CONFIG_PATH)
        secret_changed = False
        if incoming.ntrip.password:
            updated.ntrip.set_password(incoming.ntrip.password)
            secret_changed = True
        await runtime.apply_config(updated, ntrip_secret_changed=secret_changed)
        return {"ok": True, "config": public_config(updated)}


@app.post("/api/calibration/2d")
async def calibration_2d(target: Calibration2DInput) -> dict:
    async with config_write_lock:
        try:
            result = runtime.calibration_2d(target.easting_m, target.northing_m)
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        updated = replace(
            runtime.config,
            lever=replace(
                runtime.config.lever,
                forward_m=result["forward_m"],
                left_m=result["left_m"],
            ),
        )
        save_config(updated, CONFIG_PATH)
        await runtime.apply_config(updated)
        return {"ok": True, "samples": result["samples"], "lever": result}


@app.post("/api/gnss/port")
async def set_gnss_port(port: PortInput) -> dict:
    """Suelta o recupera /dev/serial0 sin reiniciar el servicio.

    Existe porque `virtual-rtk` y este servicio no pueden abrir el puerto a la
    vez: liberarlo desde aquí evita tener que entrar por SSH para el relevo.
    """
    await runtime.set_port_released(port.released)
    return {
        "ok": True,
        "released": runtime.gnss.released,
        "connected": runtime.gnss.connected,
        "port": runtime.config.gnss.port,
    }


async def _save_imu(imu: ImuConfig) -> dict:
    updated = replace(runtime.config, imu=imu)
    save_config(updated, CONFIG_PATH)
    await runtime.apply_config(updated)
    return {
        "ok": True,
        "imu": {
            "axis_mapping": list(imu.axis_mapping),
            "axis_signs": list(imu.axis_signs),
            "roll_offset_deg": imu.roll_offset_deg,
            "roll_invert": imu.roll_invert,
        },
    }


@app.post("/api/imu/zero")
async def imu_set_zero() -> dict:
    """Pone el roll a cero con la máquina nivelada.

    El operador nivela contra una referencia externa y pulsa: lo que lea el
    sensor es el desalineamiento del montaje. Se promedia una ventana, no una
    muestra: en drill-001 el acelerómetro dispersaba ±1° con el motor encendido
    y una lectura suelta habría grabado grados de error como offset permanente.
    """
    async with config_write_lock:
        window = runtime.imu.raw_roll_window()
        if window is None:
            raise HTTPException(
                status_code=409,
                detail="La IMU no está entregando datos; no se puede poner a cero",
            )
        average, spread, count = window
        required = runtime.imu.zero_samples_required
        if count < required:
            raise HTTPException(
                status_code=409,
                detail=(
                    f"faltan lecturas quietas: {count}/{required}. Deje la máquina "
                    "detenida unos segundos y repita"
                ),
            )
        if spread > runtime.config.imu.max_roll_noise_deg:
            raise HTTPException(
                status_code=409,
                detail=(
                    f"el roll dispersa ±{spread:.1f}° en la ventana: el cero saldría "
                    "de ruido. Apague el motor o revise el montaje y repita"
                ),
            )
        current = runtime.config.imu
        offset = average if current.roll_invert else -average
        result = await _save_imu(replace(current, roll_offset_deg=offset))
        result["window"] = {"average_deg": average, "spread_deg": spread, "samples": count}
        return result


@app.post("/api/imu/invert")
async def imu_invert_roll() -> dict:
    """Invierte el signo del roll. Si al inclinar a un lado el valor va al
    contrario, esto lo corrige sin tocar el mapeo de ejes."""
    async with config_write_lock:
        current = runtime.config.imu
        return await _save_imu(replace(current, roll_invert=not current.roll_invert))


@app.post("/api/imu/detect-axes")
async def imu_detect_axes() -> dict:
    """Deduce qué eje del sensor mira hacia arriba y reordena el mapeo.

    Sólo resuelve el eje vertical, que es el que hace inservible el roll cuando
    está cruzado. Los otros dos hay que verificarlos inclinando la máquina.
    """
    async with config_write_lock:
        raw = runtime.imu.raw_g
        if raw is None:
            raise HTTPException(status_code=409, detail="La IMU no está entregando datos")
        vertical = max(range(3), key=lambda i: abs(raw[i]))
        magnitude = math.sqrt(sum(v * v for v in raw))
        if magnitude < 0.8 or magnitude > 1.2:
            raise HTTPException(
                status_code=409,
                detail=(
                    f"El sensor mide {magnitude:.2f} g en vez de 1.00 g. "
                    "Deje la máquina completamente quieta y repita"
                ),
            )
        rest = [i for i in range(3) if i != vertical]
        mapping = (rest[0], rest[1], vertical)
        signs = list(runtime.config.imu.axis_signs)
        signs[2] = 1 if raw[vertical] > 0 else -1
        current = runtime.config.imu
        result = await _save_imu(
            replace(current, axis_mapping=mapping, axis_signs=tuple(signs))
        )
        result["detected"] = {
            "vertical_axis": "XYZ"[vertical],
            "gravity_g": raw[vertical],
            "magnitude_g": magnitude,
        }
        return result


@app.get("/api/v1/telemetry/stream")
async def telemetry_stream(request: Request) -> StreamingResponse:
    async def events():
        queue = runtime.subscribe()
        try:
            while not await request.is_disconnected():
                try:
                    payload = await asyncio.wait_for(queue.get(), timeout=1.0)
                except TimeoutError:
                    yield ": keepalive\n\n"
                    continue
                yield f"data: {json.dumps(payload, separators=(',', ':'))}\n\n"
        finally:
            runtime.unsubscribe(queue)

    return StreamingResponse(events(), media_type="text/event-stream")
