from dataclasses import replace
from pathlib import Path

from app.config import load_config, save_config


def test_drill_config_is_complete_and_planar() -> None:
    path = Path(__file__).resolve().parents[1] / "config.drill-001.toml"
    config = load_config(path)
    assert config.projected_crs == "EPSG:32717"
    assert not config.use_imu
    assert config.lever.forward_m == 2.270004
    assert config.lever.left_m == 1.192078
    assert config.gnss.rate_hz == 10


def test_save_roundtrip_and_password_stays_outside_config(tmp_path: Path) -> None:
    source = Path(__file__).resolve().parents[1] / "config.drill-001.toml"
    config = load_config(source)
    secret = tmp_path / "secret.json"
    config = replace(config, ntrip=replace(config.ntrip, password_file=str(secret)))
    config.ntrip.set_password("clave-no-publica")
    target = tmp_path / "config.toml"
    save_config(config, target)
    loaded = load_config(target)
    assert loaded == config
    assert "clave-no-publica" not in target.read_text(encoding="utf-8")
    assert loaded.ntrip.password() == "clave-no-publica"


def test_lever_beyond_the_physical_limit_is_refused_on_load(tmp_path: Path) -> None:
    """F-01: load_config aceptaba el brazo de 733 km que dejó escrito la calibración."""
    import pytest

    source = Path(__file__).resolve().parents[1] / "config.drill-001.toml"
    text = source.read_text(encoding="utf-8").replace(
        "forward_m = 2.270004", "forward_m = -488346.607908"
    )
    target = tmp_path / "config.toml"
    target.write_text(text, encoding="utf-8")
    with pytest.raises(ValueError, match="excede el tope"):
        load_config(target)


def test_negative_down_is_refused(tmp_path: Path) -> None:
    import pytest

    source = Path(__file__).resolve().parents[1] / "config.drill-001.toml"
    text = source.read_text(encoding="utf-8").replace("down_m = 3.919", "down_m = -1.0")
    target = tmp_path / "config.toml"
    target.write_text(text, encoding="utf-8")
    with pytest.raises(ValueError, match="negativo"):
        load_config(target)


def test_use_pitch_defaults_to_false_on_older_config_files() -> None:
    """Los config.toml anteriores no traen la clave y deben seguir cargando."""
    path = Path(__file__).resolve().parents[1] / "config.drill-001.toml"
    assert "use_pitch" not in path.read_text(encoding="utf-8")
    assert load_config(path).use_pitch is False


def test_use_pitch_survives_the_roundtrip(tmp_path: Path) -> None:
    from dataclasses import replace as dc_replace

    source = Path(__file__).resolve().parents[1] / "config.drill-001.toml"
    config = dc_replace(load_config(source), use_pitch=True)
    target = tmp_path / "config.toml"
    save_config(config, target)
    assert "use_pitch = true" in target.read_text(encoding="utf-8")
    assert load_config(target).use_pitch is True
