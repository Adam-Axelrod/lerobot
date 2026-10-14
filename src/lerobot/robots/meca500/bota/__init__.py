"""Bota force/torque sensor configs, one JSON per sensor model.

The two Meca500 benches carry different Bota sensors, and the bota_driver needs a matching
JSON (product, interface, COM port, baud rate):

    miniONE.json      BFT-MIPS-ENET-CG, Bota_Binary on COM5       - the original setup
    miniONE_pro.json  BFT-MIPS-SER-CG,  Bota_Binary_gen0 on COM8  - Huanyu's setup

Which bench uses which is decided per rig in scripts/meca500/rigs.py (BOTA_SENSORS), like the
home pose and the SpaceMouse axis signs. A COM port changes if the sensor is plugged into a
different USB socket - check Device Manager -> Ports and edit the JSON here.
"""

from pathlib import Path

BOTA_CONFIG_DIR = Path(__file__).parent

BOTA_CONFIGS: dict[str, Path] = {
    "miniONE": BOTA_CONFIG_DIR / "miniONE.json",
    "miniONE_pro": BOTA_CONFIG_DIR / "miniONE_pro.json",
}
