import yaml
import sys

motor_file = "./motor_configurations.yaml"
config_file = "./PCU_configurations.yaml"

def load_configurations():
    # Load configuration files
    try:
        # Open and read config file with info on named positions
        with open(config_file) as f:
            file = f.read()
            configurations = list(yaml.load_all(file))
            # Return all config files
            return configurations
            # FIX-YAML version on k1aoserver-new is too old.
    except:
        print("Unable to read configuration file. Shutting down.")
        sys.exit(1)

def load_motors():
    # Open and read motor file with info on valid motors
    try:
        with open(motor_file) as f: # FIX Z-STAGE
            file = f.read()
            motor_info = yaml.load(file)
            return motor_info
    except:
        print("Unable to read motor file. Shutting down.")
        sys.exit(1)