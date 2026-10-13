from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("meca500_scripted")
@dataclass
class Meca500ScriptedConfig(TeleoperatorConfig):
    # No settings: the trajectory is loaded at runtime with Meca500Scripted.load_plan(), one
    # plan per episode, by scripts/meca500/record_auto.py.
    pass
