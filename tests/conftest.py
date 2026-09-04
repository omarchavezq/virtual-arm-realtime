import os
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

# `app.main` construye el Runtime al importarse, así que necesita una config
# válida antes de que pytest recoja los módulos de prueba. Cada test que escribe
# usa la copia en tmp_path del fixture `client`; ésta sólo sirve para importar.
os.environ.setdefault("VIRTUAL_ARM_CONFIG", str(ROOT / "config.drill-001.toml"))


async def _noop() -> None:
    return None


@pytest.fixture
def config_file(tmp_path: Path) -> Path:
    """Copia de la config de drill-001 con el secreto dentro de tmp_path."""
    secret = tmp_path / "ntrip.secret"
    text = (ROOT / "config.drill-001.toml").read_text(encoding="utf-8")
    text = text.replace(
        'password_file = "/var/lib/virtual-rtk/secrets/ntrip.secret"',
        f"password_file = {str(secret)!r}",
    )
    target = tmp_path / "config.toml"
    target.write_text(text, encoding="utf-8")
    return target


@pytest.fixture
def client(config_file: Path, monkeypatch: pytest.MonkeyPatch):
    """TestClient sobre la app real, sin abrir puerto serie, IMU ni red."""
    from fastapi.testclient import TestClient

    monkeypatch.setenv("VIRTUAL_ARM_CONFIG", str(config_file))
    import app.main as main
    from app.config import load_config
    from app.io import GnssSerial, Ntrip
    from app.runtime import Runtime

    for target in (Runtime, GnssSerial, Ntrip):
        monkeypatch.setattr(target, "start", lambda self: _noop())
        monkeypatch.setattr(target, "stop", lambda self: _noop())

    monkeypatch.setattr(main, "CONFIG_PATH", config_file)
    fresh = Runtime(load_config(config_file))
    monkeypatch.setattr(main, "runtime", fresh)
    with TestClient(main.app) as test_client:
        test_client.runtime = fresh
        yield test_client


@pytest.fixture(autouse=True)
def _keep_repo_clean():
    """Ninguna prueba debe escribir en el config del repositorio."""
    source = ROOT / "config.drill-001.toml"
    original = source.read_bytes()
    yield
    assert source.read_bytes() == original, "una prueba escribió en config.drill-001.toml"
