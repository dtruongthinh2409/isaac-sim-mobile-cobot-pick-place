# Load Small Warehouse Digital Twin Environment with Tables, Metal Boxes and Objects
# Usage: ~/isaacsim-5.1.0-release/python.sh scripts/load_warehouse_env.py

import os
import sys
import json
import random

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
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
import carb

# =============================================================================
# CONFIGURATION
# =============================================================================

# Table positions (3m apart)
TABLE_1_POS = [-4.0, -3.0, 0]  # Pick station (left)
TABLE_2_POS = [-1.0, -3.0, 0]  # Place station (right)

# Table dimensions
TABLE_HEIGHT = 0.65
TABLE_TOP_THICKNESS = 0.03

# Metal box dimensions
BOX_LENGTH = 0.4
BOX_WIDTH = 0.4
BOX_HEIGHT = 0.08
WALL_THICKNESS = 0.02

# Metal box Z position (on top of table)
BOX_Z = TABLE_HEIGHT + TABLE_TOP_THICKNESS / 2 + WALL_THICKNESS / 2

# Object spawn settings
DROP_HEIGHT = 0.15  # Height above metal box to drop objects
SPAWN_MARGIN = 0.08  # Margin from metal box edges
MIN_OBJECT_DISTANCE = 0.15  # Minimum distance between objects (for gripper access)

# =============================================================================
# PATHS
# =============================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
ENVIRONMENTS_DIR = os.path.join(PROJECT_DIR, "assets", "environments")
OBJECTS_DIR = os.path.join(PROJECT_DIR, "assets", "objects")
CONFIGS_DIR = os.path.join(PROJECT_DIR, "configs")

# Environment assets
ENVIRONMENT_USD = os.path.join(ENVIRONMENTS_DIR, "small_warehouse_digital_twin.usd")
TABLE_1_USD = os.path.join(ENVIRONMENTS_DIR, "Table_1.usd")
TABLE_2_USD = os.path.join(ENVIRONMENTS_DIR, "Table_2.usd")
METAL_BOX_1_USD = os.path.join(ENVIRONMENTS_DIR, "MetalBox_1.usd")
METAL_BOX_2_USD = os.path.join(ENVIRONMENTS_DIR, "MetalBox_2.usd")

# Object assets mapping
OBJECT_USD_MAP = {
    "Red": os.path.join(OBJECTS_DIR, "cube_red.usd"),
    "Green": os.path.join(OBJECTS_DIR, "cube_green.usd"),
    "Blue": os.path.join(OBJECTS_DIR, "cube_blue.usd"),
    "TextureObject": os.path.join(OBJECTS_DIR, "chewing_gum_cool_air.usd"),
}

# Config file
INPUT_JSON = os.path.join(CONFIGS_DIR, "input.json")

print(f"Project Directory: {PROJECT_DIR}")


def load_config():
    """Load configuration from input.json"""
    if not os.path.exists(INPUT_JSON):
        print(f"WARNING: Config file not found: {INPUT_JSON}")
        return {
            "grasp_order": ["Red", "TextureObject", "Green", "Blue"],
            "start_table": 1,
            "target_table": 2
        }
    
    with open(INPUT_JSON, 'r') as f:
        config = json.load(f)
    
    print(f"Loaded config: {config}")
    return config


def add_asset(stage, usd_path, prim_path, position=None, rotation=None):
    """Add USD reference to stage with optional position and rotation (XYZ)"""
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(usd_path)
    
    if position or rotation:
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        
        if position:
            xform.AddTranslateOp().Set(Gf.Vec3d(position[0], position[1], position[2]))
        
        if rotation:
            # Rotation in degrees around X, Y, Z axes
            xform.AddRotateXYZOp().Set(Gf.Vec3f(rotation[0], rotation[1], rotation[2]))
    
    print(f"Loaded: {prim_path} -> {os.path.basename(usd_path)}")
    return prim


def get_random_spawn_position(table_pos, box_z):
    """Get random position within metal box bounds"""
    # Metal box inner area (with margin to ensure objects stay inside)
    half_length = (BOX_LENGTH / 2) - SPAWN_MARGIN
    half_width = (BOX_WIDTH / 2) - SPAWN_MARGIN
    
    x = table_pos[0] + random.uniform(-half_length, half_length)
    y = table_pos[1] + random.uniform(-half_width, half_width)
    z = box_z + BOX_HEIGHT + DROP_HEIGHT  # Above metal box
    
    return [x, y, z]


def spawn_objects(stage, config):
    """Spawn 4 objects on the start table"""
    start_table = config.get("start_table", 1)
    
    # Get table position based on start_table
    if start_table == 1:
        table_pos = TABLE_1_POS
        print(f"Spawning objects on Table 1 at {TABLE_1_POS}")
    else:
        table_pos = TABLE_2_POS
        print(f"Spawning objects on Table 2 at {TABLE_2_POS}")
    
    # Create Objects container
    stage.DefinePrim("/World/Objects", "Xform")
    
    # Spawn all 4 objects with random positions
    objects_to_spawn = ["Red", "Green", "Blue", "TextureObject"]
    spawned_positions = []
    
    for obj_name in objects_to_spawn:
        usd_path = OBJECT_USD_MAP.get(obj_name)
        if not usd_path or not os.path.exists(usd_path):
            print(f"WARNING: Object USD not found: {obj_name}")
            continue
        
        # Get random position (avoid overlapping)
        max_attempts = 10
        for _ in range(max_attempts):
            pos = get_random_spawn_position(table_pos, BOX_Z)
            
            # Check distance from other objects (for gripper access)
            too_close = False
            for prev_pos in spawned_positions:
                dist = ((pos[0] - prev_pos[0])**2 + (pos[1] - prev_pos[1])**2)**0.5
                if dist < MIN_OBJECT_DISTANCE:
                    too_close = True
                    break
            
            if not too_close:
                break
        
        spawned_positions.append(pos)
        
        # Random rotation around X, Y, Z axes
        rot_x = random.uniform(-15, 15)
        rot_y = random.uniform(-15, 15)
        rot_z = random.uniform(0, 360)
        rotation = [rot_x, rot_y, rot_z]
        
        # Prim path
        prim_path = f"/World/Objects/{obj_name}"
        
        add_asset(stage, usd_path, prim_path, pos, rotation)
        print(f"  {obj_name} at [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}], rot: ({rot_x:.1f}, {rot_y:.1f}, {rot_z:.1f})°")


def main():
    print("=" * 60)
    print("LOADING SCENE...")
    print("=" * 60)
    
    # Load config
    config = load_config()
    
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
    
    # Spawn objects based on config
    spawn_objects(stage, config)
    
    # Create Environment with defaultLight
    UsdGeom.Xform.Define(stage, "/Environment")
    distant_light = UsdLux.DistantLight.Define(stage, "/Environment/defaultLight")
    distant_light.CreateIntensityAttr(100)
    distant_light.CreateAngleAttr(0.53)
    
    # Main loop
    while simulation_app.is_running():
        simulation_app.update()
    
    simulation_app.close()


if __name__ == "__main__":
    main()
