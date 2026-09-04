from app.domain import Epoch, Heading
from app.parser import parse_unicore


def test_real_um982_r410_samples_parse_with_crc() -> None:
    heading_line = '#UNIHEADINGA,68,GPS,FINE,2433,239990400,0,0,18,13;SOL_COMPUTED,NARROW_INT,1.3868,41.3574,-0.9784,0.0000,0.1966,0.5238,"999",37,35,35,28,3,01,3,f3*18fb1b74'
    epoch_line = '#BESTNAVA,68,GPS,FINE,2433,239990400,0,0,18,9;SOL_COMPUTED,NARROW_INT,-7.89377025183,-78.13098235479,3363.0132,22.3499,WGS84,0.0153,0.0171,0.0694,"665",1.400,46.000,38,36,36,34,0,01,03,f3,SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0006,251.626423,-0.0012,0.0157,0.0096*dd35f124'

    heading = parse_unicore(heading_line)
    epoch = parse_unicore(epoch_line)

    assert isinstance(heading, Heading)
    assert heading.crc_ok
    assert heading.heading_deg == 41.3574
    assert heading.baseline_m == 1.3868
    assert isinstance(epoch, Epoch)
    assert epoch.crc_ok
    assert epoch.fix == "FIXED"
    assert epoch.ellipsoidal_height_m == 3385.3631
