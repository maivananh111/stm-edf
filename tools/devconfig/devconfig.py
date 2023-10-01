
import os
import sys
import re
import logging


project_cproject_file = ".cproject"
dev_config_suffix     = ".Kconfig"

set_dev_config_file   = f"set_dev_config{dev_config_suffix}"
dev_config_file       = f"_dev_config{dev_config_suffix}"
project_config_dir    = "buildconfig"
out_config_file       = "devconfig.conf"
header_kconfig_file   = "devconfig.h"
config_tool           = "guiconfig"

logging.basicConfig(level=logging.INFO, format='[%(asctime)s] %(levelname)s: %(message)s')

def get_proj_device_name(cproject_file_path):
    try:
        with open(cproject_file_path, 'r') as file:
            content = file.read()
            match = re.search(r'STM32[^"]*', content)
            if match:
                return match.group()
            else:
                return None
            
    except FileNotFoundError:
        logging.error(f"{cproject_file_path} -> No such file in directory.")
        return None
    
def find_and_set_dev_config_files(project_path, config_path):
    conf_files = []
    device_name = get_proj_device_name(f"{project_path}/{project_cproject_file}")
    logging.info(f"Configuration device: {device_name}")

    for root, dirs, files in os.walk(project_path): # Search all folder.
        for file in files: # Search all file in folder.
            if file.endswith(dev_config_suffix) and file != set_dev_config_file:
                conf_files.append(f"source \"{os.path.join(root, file)}\"\n")

    with open(config_path, "w") as f:
        f.write(f"source \"{edf_path}/components/devconfig/device/{device_name[5:7].lower()}{dev_config_file}\"\n")
        f.write(f"source \"{edf_path}/components/devconfig/drivers.Kconfig\"\n")
        f.write(f"source \"{edf_path}/components/devconfig/features.Kconfig\"\n")
        f.write(f"source \"{edf_path}/components/devconfig/freertos.Kconfig\"\n")
        for conf_file in conf_files:
            f.write(conf_file)
    logging.info("Listed all config file.")


if __name__ == "__main__":
    logging.info("Device configuration tool statting up.")
    # Get EDF_PATH.
    edf_path = os.environ.get('EDF_PATH')
    if edf_path:
        logging.info(f"EDF_PATH: {edf_path}")
    else:
        logging.error("EDF_PATH environment variable not found.")

    # Get first arguments (project directory).
    project_dir = sys.argv[1]
    logging.info("Project directory:" + project_dir)

    # Create set kconfig file path.
    set_dev_config_dir = f"{project_dir}/{project_config_dir}/{set_dev_config_file}"
    out_config_dir = f"{project_dir}/{project_config_dir}/{out_config_file}"
    header_dir = f"{project_dir}/{project_config_dir}/{header_kconfig_file}"

    logging.info("Set devconfig file directory:" + set_dev_config_dir)
    logging.info("Output config file directory:" + out_config_dir)
    logging.info("Output header config file directory:" + header_dir)

    # Search all .kconf file.
    find_and_set_dev_config_files(project_dir, set_dev_config_dir)
    
    # Declare output config file.
    os.environ["KCONFIG_CONFIG"] = out_config_dir

    # Open kconfig gui.
    os.system(f"{config_tool} {set_dev_config_dir}")
    
    # Generate header config file.
    os.system(f"genconfig {set_dev_config_dir} --header-path {header_dir}")
