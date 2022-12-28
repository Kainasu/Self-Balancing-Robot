import pathlib

OUTPUT_DIR = pathlib.Path("ai/out")
TEST_DIR   = pathlib.Path("TEST")
CONFIG_DIR = pathlib.Path("ai/config")
MODEL_DIR  = pathlib.Path("ai/models")

EXPERIMENT_CONFIG_FILENAME  = pathlib.Path("config_ppo.yaml")
ENVIRONMENT_CONFIG_FILENAME = pathlib.Path("config_env.yaml")
PERFORMANCE_REPORT_FILENAME = pathlib.Path("performances.yaml")

if __name__ == '__main__':
    print(f"EXPERIMENT_CONFIG_FILENAME: <{EXPERIMENT_CONFIG_FILENAME}>")
    print(f"ENVIRONMENT_CONFIG_FILENAME: <{ENVIRONMENT_CONFIG_FILENAME}>")
    print(f"PERFORMANCE_REPORT_FILENAME: <{PERFORMANCE_REPORT_FILENAME}>")

