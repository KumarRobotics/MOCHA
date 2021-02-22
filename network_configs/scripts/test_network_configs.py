#!/usr/bin/env python3
import rospkg
import os
import yaml

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('network_configs')
    config_file = "networkConfigs.yml"
    yaml_path = os.path.join(package_path, "config", config_file)
    with open(yaml_path, 'r') as f:
        cfg = yaml.load(f)
    print(cfg)
