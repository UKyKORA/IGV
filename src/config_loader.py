import yaml

def load_yaml_config(filename):
    cfg = None
    with open(filename, 'r') as config_file:
        cfg = yaml.load(config_file)
    return cfg