import logging
from pathlib import Path
import yaml

class SimulationTestLogger:
    def __init__(self, algorithm: str, map_name: str, config_algo_path):
        self.algorithm = algorithm
        self.map_name = map_name
        self.config_algo_path = config_algo_path
        Path("logs").mkdir(exist_ok=True)

        self.logger = logging.getLogger(
            f"sim_tests.{algorithm}.{map_name}"
        )
        self.logger.propagate = False
        self.logger.setLevel(logging.INFO)
        #configures where we're outputting our logs
        handler = logging.FileHandler(
            f"logs/{map_name}_{algorithm}.log",
            mode="w"
        )
        handler.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)s] %(message)s"
        ))

        #iterate over old stale handlers that may still exist if we're instantiating our logger is instantiated multiple times 
        #this is because python returns an existing logger if one with the same name already exists.
        for old_handler in self.logger.handlers[:]:
            self.logger.removeHandler(old_handler)
            old_handler.close()

        self.logger.addHandler(handler)

        
    def log_test_description(self, test_description):
        #open config/algorithms.yaml file 
        with open(self.config_algo_path) as file:
            config = yaml.safe_load(file)

        #specify algorithm 
        algorithms = config["algorithms"]
        selected_algo = algorithms[self.algorithm]
        
        #use the data from the yaml to configure the algorithms parameters
        params = dict(selected_algo.get("parameters", {}))

        #for pure pursuit specifically we make sure to run the proper waypoints depending on the map being used
        if(selected_algo["package"] == "pure_pursuit_a"): 
            params["track"] = selected_algo["tracks"][self.map_name]
        
        self.logger.info(
            f"Test Started: Testing {self.algorithm} in {self.map_name}" +
            "".join(
                    f"\n\t{key}: {value}"
                    for key, value in params.items()
            ) +  f"\n\tDescription: ${test_description}"
        )

    def log_crash(self, reason: str):
        self.logger.error(f"\n\tCrash detected: {reason}")

    def log_lap(self, lap: int, duration: float):
        self.logger.info(f"\n\tLap {lap} completed in {duration:.2f}s")

    def log_result(self, passed: bool):
        if passed:
            self.logger.info(f"Result: TEST PASSED")
        else:
            self.logger.info(f"Result: TEST FAILED")
    
        
    
