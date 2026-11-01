import logging
from pathlib import Path

class SimulationTestLogger:
    def __init__(self, algorithm: str, map_name: str, min_speed: int, max_speed: int):
        self.algorithm = algorithm
        self.map_name = map_name
        self.min_speed = min_speed
        self.max_speed = max_speed
        Path("logs").mkdir(exist_ok=True)

        self.logger = logging.getLogger(
            f"sim_tests.{algorithm}.{map_name}"
        )
        self.logger.propagate = False
        self.logger.setLevel(logging.INFO)

        handler = logging.FileHandler(
            f"logs/{map_name}_{algorithm}.log",
            mode="w"
        )
        handler.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)s] %(message)s"
        ))
        for old_handler in self.logger.handlers[:]:
            self.logger.removeHandler(old_handler)
            old_handler.close()

        self.logger.addHandler(handler)

    def log_info(self):
        self.logger.info(
            f"### Test Suite: Testing {self.algorithm} in {self.map_name} ###"
            f"\n\tmin_speed set to: {self.min_speed}"
            f"\n\tmax_speed set to {self.max_speed}"
        )

    def log_test_description(self, test_description):
        self.logger.info(f"\n\tTest Description: ${test_description}")

    def log_crash(self, reason: str):
        self.logger.error(f"\n\tCrash detected: {reason}")

    def log_lap(self, lap: int, duration: float):
        self.logger.info(f"\n\tLap {lap} completed in {duration:.2f}s")

    def log_result(self, passed: bool):
        if passed:
            self.logger.info(f"### TEST PASSED ###")
        else:
            self.logger.info(f"### TEST FAILED ###")
        
    
