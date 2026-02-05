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
import omni.kit.app
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
import carb

# =============================================================================
# ENABLE OMNIGRAPH AND ROS2 EXTENSIONS
# =============================================================================

EXTENSIONS_TO_ENABLE = [
    # OmniGraph core extensions
    "omni.graph.core",
    "omni.graph.action",
    "omni.graph.nodes",
    "omni.graph.scriptnode",
    "omni.graph.window.action",      # Action Graph Editor window
    "omni.graph.window.generic",     # Visual Scripting window
    # ROS2 Bridge extensions (requires ROS2 env vars set before starting)
    "isaacsim.ros2.bridge",
]

def enable_extensions():
    """Enable required OmniGraph and ROS2 extensions"""
    print("=" * 60)
    print("Enabling OmniGraph and ROS2 extensions...")
    print("=" * 60)
    
    manager = omni.kit.app.get_app().get_extension_manager()
    
    for ext_name in EXTENSIONS_TO_ENABLE:
        try:
            manager.set_extension_enabled_immediate(ext_name, True)
            print(f"  ✓ Enabled: {ext_name}")
        except Exception as e:
            print(f"  ✗ Failed to enable {ext_name}: {e}")
    
    # Wait for extensions to load
    for _ in range(10):
        simulation_app.update()
    
    print("Extensions enabled successfully!")

# Enable extensions
enable_extensions()

# =============================================================================
# CONFIGURATION - Load from .env file
# =============================================================================

def load_env_config():
    """Load configuration from .env file"""
    env_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs", ".env")
    config = {}
    
    if os.path.exists(env_path):
        with open(env_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    key, value = line.split('=', 1)
                    try:
                        config[key.strip()] = float(value.strip())
                    except ValueError:
                        config[key.strip()] = value.strip()
    return config

# Load from .env
env_config = load_env_config()

# Table positions (3m apart)
TABLE_1_POS = [
    env_config.get('TABLE_1_X', -4.0),
    env_config.get('TABLE_1_Y', -3.0),
    env_config.get('TABLE_1_Z', 0.0)
]  # Pick station (left)
TABLE_2_POS = [
    env_config.get('TABLE_2_X', -1.0),
    env_config.get('TABLE_2_Y', -3.0),
    env_config.get('TABLE_2_Z', 0.0)
]  # Place station (right)

# Table dimensions
TABLE_HEIGHT = env_config.get('TABLE_HEIGHT', 0.65)
TABLE_TOP_THICKNESS = env_config.get('TABLE_TOP_THICKNESS', 0.03)

# Metal box dimensions
BOX_LENGTH = env_config.get('BOX_LENGTH', 0.4)
BOX_WIDTH = env_config.get('BOX_WIDTH', 0.4)
BOX_HEIGHT = env_config.get('BOX_HEIGHT', 0.08)
WALL_THICKNESS = env_config.get('WALL_THICKNESS', 0.02)

# Metal box Z position (on top of table)
BOX_Z = TABLE_HEIGHT + TABLE_TOP_THICKNESS / 2 + WALL_THICKNESS / 2

# Object spawn settings
DROP_HEIGHT = env_config.get('DROP_HEIGHT', 0.15)
SPAWN_MARGIN = env_config.get('SPAWN_MARGIN', 0.08)
MIN_OBJECT_DISTANCE = env_config.get('MIN_OBJECT_DISTANCE', 0.15)

# =============================================================================
# ROBOT CONFIGURATION
# =============================================================================

# Distance in front of table for robot positioning (in meters)
ROBOT_DISTANCE_FROM_TABLE = env_config.get('ROBOT_DISTANCE_FROM_TABLE', 0.5)

# Robot initial Z position (ground level)
ROBOT_Z = env_config.get('ROBOT_Z', 0.0)

# Robot positions in front of each table
# Robot faces the table (rotated 90 degrees to face negative Y direction)
ROBOT_AT_TABLE_1 = [TABLE_1_POS[0], TABLE_1_POS[1] + ROBOT_DISTANCE_FROM_TABLE, ROBOT_Z]
ROBOT_AT_TABLE_2 = [TABLE_2_POS[0], TABLE_2_POS[1] + ROBOT_DISTANCE_FROM_TABLE, ROBOT_Z]

# Robot rotation to face the table (facing -Y direction, so rotate 270 degrees around Z)
ROBOT_FACING_TABLE_ROTATION = [0, 0, -90]  # Degrees: facing -Y direction

# Navigation goal positions (for Nav2)
NAV_GOAL_TABLE_1 = {
    "position": {"x": ROBOT_AT_TABLE_1[0], "y": ROBOT_AT_TABLE_1[1], "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}  # Quaternion facing -Y
}
NAV_GOAL_TABLE_2 = {
    "position": {"x": ROBOT_AT_TABLE_2[0], "y": ROBOT_AT_TABLE_2[1], "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": -0.707, "w": 0.707}  # Quaternion facing -Y
}

# =============================================================================
# PATHS
# =============================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
ENVIRONMENTS_DIR = os.path.join(PROJECT_DIR, "assets", "environments")
OBJECTS_DIR = os.path.join(PROJECT_DIR, "assets", "objects")
ROBOTS_DIR = os.path.join(PROJECT_DIR, "assets", "robots")
CONFIGS_DIR = os.path.join(PROJECT_DIR, "configs")

# Environment assets
ENVIRONMENT_USD = os.path.join(ENVIRONMENTS_DIR, "small_warehouse_digital_twin.usd")
TABLE_1_USD = os.path.join(ENVIRONMENTS_DIR, "Table_1.usd")
TABLE_2_USD = os.path.join(ENVIRONMENTS_DIR, "Table_2.usd")
METAL_BOX_1_USD = os.path.join(ENVIRONMENTS_DIR, "MetalBox_1.usd")
METAL_BOX_2_USD = os.path.join(ENVIRONMENTS_DIR, "MetalBox_2.usd")

# Robot asset (Mobile Manipulator with ROS2 OmniGraph)
ROBOT_USD = os.path.join(ROBOTS_DIR, "MobileManipulator_with_ros2.usd")

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


def spawn_robot(stage, config):
    """Spawn Mobile Manipulator robot at the start table position"""
    start_table = config.get("start_table", 1)
    target_table = config.get("target_table", 2)
    
    # Determine robot spawn position based on start_table
    if start_table == 1:
        robot_pos = ROBOT_AT_TABLE_1.copy()
        robot_rot = ROBOT_FACING_TABLE_ROTATION.copy()
        nav_goal_start = NAV_GOAL_TABLE_1
        nav_goal_target = NAV_GOAL_TABLE_2
        print(f"Robot starting at Table 1: position {robot_pos}")
    else:
        robot_pos = ROBOT_AT_TABLE_2.copy()
        robot_rot = ROBOT_FACING_TABLE_ROTATION.copy()
        nav_goal_start = NAV_GOAL_TABLE_2
        nav_goal_target = NAV_GOAL_TABLE_1
        print(f"Robot starting at Table 2: position {robot_pos}")
    
    # Check if robot USD exists
    if not os.path.exists(ROBOT_USD):
        print(f"ERROR: Robot USD not found: {ROBOT_USD}")
        return None
    
    # Load robot
    robot_prim = add_asset(stage, ROBOT_USD, "/World/MobileManipulator", robot_pos, robot_rot)
    
    print("=" * 60)
    print("ROBOT SPAWN INFO:")
    print(f"  Start Table: {start_table}")
    print(f"  Target Table: {target_table}")
    print(f"  Robot Position: x={robot_pos[0]:.2f}, y={robot_pos[1]:.2f}, z={robot_pos[2]:.2f}")
    print(f"  Robot Rotation: {robot_rot}")
    print("=" * 60)
    print("NAVIGATION GOALS (for Nav2):")
    print(f"  Goal Table 1: {NAV_GOAL_TABLE_1}")
    print(f"  Goal Table 2: {NAV_GOAL_TABLE_2}")
    print("=" * 60)
    
    return robot_prim


def get_navigation_goals(config):
    """Get navigation goal positions based on config"""
    start_table = config.get("start_table", 1)
    target_table = config.get("target_table", 2)
    
    goals = {
        "start": NAV_GOAL_TABLE_1 if start_table == 1 else NAV_GOAL_TABLE_2,
        "target": NAV_GOAL_TABLE_1 if target_table == 1 else NAV_GOAL_TABLE_2,
        "table_1": NAV_GOAL_TABLE_1,
        "table_2": NAV_GOAL_TABLE_2,
    }
    
    return goals


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
    
    # Spawn robot at start table
    spawn_robot(stage, config)
    
    # Get navigation goals for later use
    nav_goals = get_navigation_goals(config)
    print(f"Navigation goals stored: {list(nav_goals.keys())}")
    
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
