# Load Small Warehouse Digital Twin Environment
# Usage: ~/isaacsim-5.1.0-release/python.sh scripts/load_warehouse_env.py

import os
import sys

from isaacsim import SimulationApp

CONFIG = {
    "headless": False,
    "width": 1920,
    "height": 1080,
    "window_width": 1920,
    "window_height": 1080,
    "anti_aliasing": 1,
    "renderer": "RayTracedLighting",
}

print("=" * 60)
print("STARTING ISAAC SIM...")
print("=" * 60)

simulation_app = SimulationApp(launch_config=CONFIG)

# Import after SimulationApp is created
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import carb

# Environment path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
ENVIRONMENT_USD = os.path.join(
    PROJECT_DIR, "assets", "environments", "small_warehouse_digital_twin.usd"
)

print(f"Project Directory: {PROJECT_DIR}")
print(f"Environment USD: {ENVIRONMENT_USD}")


def main():
    print("=" * 60)
    print("LOADING ENVIRONMENT...")
    print("=" * 60)
    
    if not os.path.exists(ENVIRONMENT_USD):
        carb.log_error(f"File not found: {ENVIRONMENT_USD}")
        simulation_app.close()
        sys.exit(1)
    
    world = World(stage_units_in_meters=1.0)
    
    add_reference_to_stage(usd_path=ENVIRONMENT_USD, prim_path="/World")
    
    print(f"Loaded: {ENVIRONMENT_USD}")
    
    world.reset()
    
    print("=" * 60)
    print("ENVIRONMENT LOADED SUCCESSFULLY!")
    print("Press PLAY (Space) to run simulation")
    print("Close window to exit")
    print("=" * 60)
    
    while simulation_app.is_running():
        world.step(render=True)
    
    simulation_app.close()


if __name__ == "__main__":
    main()
