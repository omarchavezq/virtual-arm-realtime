from __future__ import annotations

import asyncio
import base64
import contextlib
import logging
from collections.abc import Awaitable, Callable

from app.config import GnssConfig, NtripConfig

log = logging.getLogger(__name__)


class GnssSerial:
    def __init__(self, config: GnssConfig, on_line: Callable[[str], Awaitable[None]]) -> None:
        self.config = config
        self.on_line = on_line
        self.reader = None
        self.writer = None
        self._task: asyncio.Task | None = None
        self._write_lock = asyncio.Lock()
        self.connected = False
        # Liberado a propósito por el operador, para que otro proceso pueda abrir
        # el puerto. No se persiste: un reinicio siempre vuelve al estado normal.
        self.released = False

    async def start(self) -> None:
        if self._task and not self._task.done():
            return
        self._task = asyncio.create_task(self._run(), name="gnss-serial")

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None
        if self.writer:
            self.writer.close()
        # Sin esto, write() seguiría escribiendo sobre un writer ya cerrado.
        self.reader = self.writer = None
        self.connected = False

    async def release(self) -> None:
        """Suelta el puerto serie sin detener el servicio."""
        await self.stop()
        self.released = True

    async def resume(self) -> None:
        # La bandera es de release/resume, no un efecto secundario de crear la
        # tarea: si start() no llega a ejecutarse, el estado seguiría mintiendo.
        self.released = False
        await self.start()

    async def write(self, data: bytes) -> None:
        async with self._write_lock:
            if self.writer is None:
                raise ConnectionError("GNSS no conectado")
            self.writer.write(data)
            await self.writer.drain()

    async def _run(self) -> None:
        import serial_asyncio

        backoff = 0.5
        while True:
            try:
                self.reader, self.writer = await serial_asyncio.open_serial_connection(
                    url=self.config.port, baudrate=self.config.baudrate
                )
                self.connected = True
                period = f"{1 / self.config.rate_hz:g}"
                commands = [
                    "UNLOGALL",
                    "CONFIG RTK TIMEOUT 10",
                    "CONFIG DGPS TIMEOUT 10",
                    f"BESTNAVA {period}",
                    f"UNIHEADINGA {period}",
                    f"GPGGA {period}",
                    "GPRMC 1",
                    "SAVECONFIG",
                ]
                await self.write("".join(f"{c}\r\n" for c in commands).encode("ascii"))
                backoff = 0.5
                while True:
                    line = await self.reader.readline()
                    if not line:
                        raise ConnectionError("puerto GNSS cerrado")
                    text = line.decode("ascii", errors="replace").strip("\r\n\x00 ")
                    if text:
                        await self.on_line(text)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001 - reconexión ante cualquier fallo serial
                self.connected = False
                self.reader = self.writer = None
                log.warning("GNSS: %s; reconexión en %.1f s", exc.__class__.__name__, backoff)
                await asyncio.sleep(backoff)
                backoff = min(10.0, backoff * 2)


class Ntrip:
    def __init__(
        self,
        config: NtripConfig,
        write_rtcm: Callable[[bytes], Awaitable[None]],
        get_gga: Callable[[], str | None],
    ) -> None:
        self.config = config
        self.write_rtcm = write_rtcm
        self.get_gga = get_gga
        self.state = "DISCONNECTED"
        self._task: asyncio.Task | None = None
        self._gga_task: asyncio.Task | None = None

    async def start(self) -> None:
        self._task = asyncio.create_task(self._run(), name="ntrip")

    async def stop(self) -> None:
        for task in (self._gga_task, self._task):
            if task:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await task
        self.state = "DISCONNECTED"

    async def _run(self) -> None:
        backoff = 1.0
        while True:
            writer = None
            try:
                self.state = "CONNECTING"
                reader, writer = await asyncio.open_connection(self.config.host, self.config.port)
                password = self.config.password()
                token = base64.b64encode(f"{self.config.username}:{password}".encode()).decode()
                request = (
                    f"GET /{self.config.mountpoint.lstrip('/')} HTTP/1.0\r\n"
                    f"Host: {self.config.host}:{self.config.port}\r\n"
                    "User-Agent: NTRIP virtual-arm-realtime/0.1\r\n"
                    f"Authorization: Basic {token}\r\n\r\n"
                )
                writer.write(request.encode("ascii"))
                await writer.drain()
                header = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), timeout=10)
                first = header.splitlines()[0].decode("ascii", errors="replace")
                if "200" not in first:
                    raise ConnectionError(f"NTRIP rechazado: {first[:80]}")
                self.state = "CONNECTED"
                backoff = 1.0
                self._gga_task = asyncio.create_task(self._send_gga(writer), name="ntrip-gga")
                while True:
                    data = await asyncio.wait_for(reader.read(4096), timeout=30)
                    if not data:
                        raise ConnectionError("caster cerró la conexión")
                    await self.write_rtcm(data)
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001 - reconexión ante cualquier fallo del caster
                self.state = "ERROR"
                log.warning("NTRIP: %s; reconexión en %.1f s", exc.__class__.__name__, backoff)
            finally:
                if self._gga_task:
                    self._gga_task.cancel()
                    with contextlib.suppress(asyncio.CancelledError):
                        await self._gga_task
                    self._gga_task = None
                if writer:
                    writer.close()
                    with contextlib.suppress(Exception):
                        await writer.wait_closed()
            await asyncio.sleep(backoff)
            backoff = min(30.0, backoff * 2)

    async def _send_gga(self, writer: asyncio.StreamWriter) -> None:
        while True:
            gga = self.get_gga()
            if gga:
                writer.write((gga + "\r\n").encode("ascii"))
                await writer.drain()
            await asyncio.sleep(self.config.gga_interval_s)
