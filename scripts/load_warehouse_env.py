# Load Small Warehouse Digital Twin Environment with Tables and Metal Boxes
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
}

print("=" * 60)
print("Loading warehouse environment...")
print("=" * 60)

simulation_app = SimulationApp(launch_config=CONFIG)

# Import after SimulationApp is created
import omni.usd
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Gf, Usd, UsdGeom, UsdLux
import carb

# ENV

# Table positions
TABLE_1_POS = [-4.0, -3.0, 0]  # Pick station (left)
TABLE_2_POS = [-1.0, -3.0, 0]   # Place station (right, 3m apart)

# Table dimensions (for calculating metal box height)
TABLE_HEIGHT = 0.65
TABLE_TOP_THICKNESS = 0.03
WALL_THICKNESS = 0.02

# Metal box Z position (on top of table)
BOX_Z = TABLE_HEIGHT + TABLE_TOP_THICKNESS / 2 + WALL_THICKNESS / 2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
ENVIRONMENTS_DIR = os.path.join(PROJECT_DIR, "assets", "environments")

ENVIRONMENT_USD = os.path.join(ENVIRONMENTS_DIR, "small_warehouse_digital_twin.usd")
TABLE_1_USD = os.path.join(ENVIRONMENTS_DIR, "Table_1.usd")
TABLE_2_USD = os.path.join(ENVIRONMENTS_DIR, "Table_2.usd")
METAL_BOX_1_USD = os.path.join(ENVIRONMENTS_DIR, "MetalBox_1.usd")
METAL_BOX_2_USD = os.path.join(ENVIRONMENTS_DIR, "MetalBox_2.usd")

print(f"Project Directory: {PROJECT_DIR}")


def add_asset(stage, usd_path, prim_path, position=None):
    """Add USD reference to stage with optional position"""
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(usd_path)
    
    if position:
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(position[0], position[1], position[2]))
    
    print(f"Loaded: {prim_path} -> {os.path.basename(usd_path)}")


def main():
    print("Loading warehouse environment...")
    print("=" * 60)
    
    # Create new stage
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    
    # Set stage metadata (meters)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    # Create /World as defaultPrim
    world_prim = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(world_prim)
    
    # Load warehouse environment
    add_asset(stage, ENVIRONMENT_USD, "/World/small_warehouse_digital_twin")
    
    # Load tables
    add_asset(stage, TABLE_1_USD, "/World/Table_1", TABLE_1_POS)
    add_asset(stage, TABLE_2_USD, "/World/Table_2", TABLE_2_POS)
    
    # Load metal boxes (on top of tables)
    metal_box_1_pos = [TABLE_1_POS[0], TABLE_1_POS[1], BOX_Z]
    metal_box_2_pos = [TABLE_2_POS[0], TABLE_2_POS[1], BOX_Z]
    
    add_asset(stage, METAL_BOX_1_USD, "/World/MetalBox_1", metal_box_1_pos)
    add_asset(stage, METAL_BOX_2_USD, "/World/MetalBox_2", metal_box_2_pos)
    
    # Create Environment with defaultLight
    UsdGeom.Xform.Define(stage, "/Environment")
    distant_light = UsdLux.DistantLight.Define(stage, "/Environment/defaultLight")
    distant_light.CreateIntensityAttr(100)
    distant_light.CreateAngleAttr(0.53)
    print("Created: /Environment/defaultLight")
    
    # Main loop
    while simulation_app.is_running():
        simulation_app.update()
    
    simulation_app.close()


if __name__ == "__main__":
    main()
