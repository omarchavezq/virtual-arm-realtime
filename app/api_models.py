from pydantic import BaseModel, ConfigDict, Field

from app.config import LEVER_LIMIT_M


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class NtripInput(StrictModel):
    host: str = Field(min_length=1, max_length=255)
    port: int = Field(ge=1, le=65535)
    mountpoint: str = Field(min_length=1, max_length=128)
    username: str = Field(min_length=1, max_length=128)
    password: str | None = Field(default=None, max_length=256)


class CoordinatesInput(StrictModel):
    projected_crs: str = Field(min_length=4, max_length=128)


class LeverInput(StrictModel):
    forward_m: float = Field(ge=-LEVER_LIMIT_M, le=LEVER_LIMIT_M, allow_inf_nan=False)
    left_m: float = Field(ge=-LEVER_LIMIT_M, le=LEVER_LIMIT_M, allow_inf_nan=False)
    down_m: float = Field(ge=0, le=LEVER_LIMIT_M, allow_inf_nan=False)


class CalculationInput(StrictModel):
    use_imu: bool
    use_pitch: bool = False


class ConfigInput(StrictModel):
    ntrip: NtripInput
    coordinates: CoordinatesInput
    lever: LeverInput
    calculation: CalculationInput


class Calibration2DInput(StrictModel):
    # Sin tope de rango: una coordenada de grilla válida puede ser grande. El
    # control real es el brazo resultante, que se verifica en la respuesta.
    easting_m: float = Field(allow_inf_nan=False)
    northing_m: float = Field(allow_inf_nan=False)



class PortInput(StrictModel):
    """Control manual del puerto serie. No se persiste: un reinicio del servicio
    siempre vuelve al puerto conectado."""

    released: bool
