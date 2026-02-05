#!/usr/bin/env python3
"""
Generate AMCL initial pose based on robot spawn config.
Reads from configs/input.json and outputs YAML params for Nav2.
"""

import os
import sys
import json

def load_env_config():
    """Load configuration from .env file"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(script_dir, "..", "..", "..", "..", ".."))
    env_path = os.path.join(project_root, "configs", ".env")
    
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

# Robot configuration (loaded from .env, with defaults as fallback)
TABLE_1_POS = [
    env_config.get('TABLE_1_X', -4.0),
    env_config.get('TABLE_1_Y', -3.0),
    env_config.get('TABLE_1_Z', 0.0)
]
TABLE_2_POS = [
    env_config.get('TABLE_2_X', -1.0),
    env_config.get('TABLE_2_Y', -3.0),
    env_config.get('TABLE_2_Z', 0.0)
]
ROBOT_DISTANCE_FROM_TABLE = env_config.get('ROBOT_DISTANCE_FROM_TABLE', 0.5)
ROBOT_YAW = env_config.get('ROBOT_YAW', -1.57079)

# Robot positions
ROBOT_AT_TABLE_1 = [TABLE_1_POS[0], TABLE_1_POS[1] + ROBOT_DISTANCE_FROM_TABLE, 0.0]
ROBOT_AT_TABLE_2 = [TABLE_2_POS[0], TABLE_2_POS[1] + ROBOT_DISTANCE_FROM_TABLE, 0.0]


def load_config():
    """Load robot spawn configuration"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
    config_path = os.path.join(project_dir, "configs", "input.json")
    
    if not os.path.exists(config_path):
        print(f"ERROR: Config not found: {config_path}", file=sys.stderr)
        return {"start_table": 1}  # Default
    
    with open(config_path, 'r') as f:
        return json.load(f)


def get_initial_pose(config):
    """Get initial pose based on start_table config"""
    start_table = config.get("start_table", 1)
    
    if start_table == 1:
        pos = ROBOT_AT_TABLE_1
    else:
        pos = ROBOT_AT_TABLE_2
    
    return {
        'x': pos[0],
        'y': pos[1],
        'z': pos[2],
        'yaw': ROBOT_YAW
    }


def main():
    """Output initial pose as YAML format for Nav2"""
    config = load_config()
    pose = get_initial_pose(config)
    start_table = config.get("start_table", 1)
    
    # Print YAML format (can be piped to params file or used in launch)
    print(f"# Auto-generated initial pose for robot starting at Table {start_table}")
    print(f"initial_pose:")
    print(f"  x: {pose['x']}")
    print(f"  y: {pose['y']}")
    print(f"  z: {pose['z']}")
    print(f"  yaw: {pose['yaw']}")
    
    # Also print Python dict for launch file
    print(f"\n# Python dict format:")
    print(f"# {{'x': {pose['x']}, 'y': {pose['y']}, 'z': {pose['z']}, 'yaw': {pose['yaw']}}}")


if __name__ == "__main__":
    main()
