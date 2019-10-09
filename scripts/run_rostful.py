#!/usr/bin/env python
import subprocess
import argparse
import rospkg
import rospy

def main():

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # Get default config file path
    config_path_default = rospack.get_path("inspector_gcs") + "/config/rostful.cfg"
    # config_path_default = rospack.get_path("rostful_launcher") + "/config/rostful.cfg"

    # Parse arguments
    parser = argparse.ArgumentParser(description='Run rostful script')
    parser.add_argument('-c', type=str, default=config_path_default,
                        help='config file path')
    args, unknown = parser.parse_known_args()

    # Run rostful
    rostful_args = "python -m rostful run -c " + args.c

    try:
        subprocess.call(rostful_args, shell=True)
    except KeyboardInterrupt:
        pass
    finally:
        pass


if __name__ == "__main__":
    main()