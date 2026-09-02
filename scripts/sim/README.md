# Isaac Sim / Isaac Lab workspace

Scripts for working on the Meca500 **in simulation**. Anything that talks to the real
arm or the real cameras lives in [`../meca500/`](../meca500/) instead.

## These do NOT run in the hardware `.venv`

Isaac Sim ships its own Python. The repo's `.venv` is for the real robot only and has
no `isaaclab`, so running these scripts with it fails at import. Use either:

```powershell
python sim.py                    # launcher: finds the Isaac interpreter for you
```

```powershell
& "$env:LOCALAPPDATA\miniconda3\envs\env_isaaclab\python.exe" load_meca500_sim.py
```

`sim.py` looks for the interpreter in this order: `$MECA_SIM_PYTHON`, the current
interpreter if `isaaclab` already imports, then the usual conda locations. Set
`MECA_SIM_PYTHON` if your env lives somewhere unusual.

## Contents

| Script | Purpose |
|---|---|
| [sim.py](sim.py) | Launcher. Re-runs `load_meca500_sim.py` under the Isaac Sim Python and forwards any extra flags (`--test`, `--headless`, `--steps N`, `--jog-gain`, `--deadzone`) |
| [load_meca500_sim.py](load_meca500_sim.py) | Loads the Meca500 into Isaac Sim as an Isaac Lab `Articulation` on a ground plane with the desk scene, and jogs its 6 joints with a SpaceMouse |

## Gotchas worth knowing

- **Assets:** the arm is [`mecademic_description/urdf/meca_arm_only/`](../../mecademic_description/urdf/meca_arm_only/) (6 revolute joints, no gripper). The full `urdf/meca/meca.usd` export is an empty 492-byte stub — do not use it.
- **Joint drive:** joints go through the tensor API (`set_joint_position_target`), not the GUI drive-target slider, so the GPU pipeline works and `--device cpu` is unnecessary.
- **`hidapi.dll`:** not bundled with the Isaac env, so the script reuses the copy in the hardware `.venv` and prepends it to `PATH` — easyhid resolves it via `ctypes.util.find_library()`, which ignores `os.add_dll_directory`.
- **Teardown:** `simulation_app.close()` can hang indefinitely on Windows during CUDA/physics teardown, so the script ends with `os._exit(0)`.

Both scripts locate the repo root as `Path(__file__).resolve().parents[2]`, so they must
stay two levels below it (`scripts/sim/`) for the asset paths to resolve.
