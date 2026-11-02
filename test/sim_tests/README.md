# Simulator Tests

These are full-system tests that sequentially test out algorithm nodes (pure_pursuit, disparity_extender, etc.) in the simulator.

# Data Driven Testing

[Data Driven Testing](https://www.geeksforgeeks.org/software-testing/data-driven-testing/) is the testing methodology being used for these simulator tests,
where we are essentially reusing the same testing script for differing combinations of chosen maps and algorithms (ftg in levine as an example). This allows us to seperate the implementation of the actual test logic from the data being passed in (hence the "data driven" name).

# In depth explanation

## 1. The shell script
The shell script lets you specify the algorithms from the config/algorithms.yaml you want to pass in via an array you can modify containg the algorithm names.
It also allows you to specify the names of the maps you want to test your algorithm nodes in. You'll notice the nested loop that iterates upon both of these arrays, first selecting a map, then going through each algorithm. 

Importantly you should ensure that the algorithms in the array exist as well in config/algorithms.yaml. Otherwise running the tests will NOT work.

Something interesting to note is how we are configuring the map, which is done using a stream editor command, which directly edits the sim.yaml file to use a different png file from the simulator folder (for example levine.png, or Spielberg.png).

## 2. The config/algorithms.yaml file
This file holds importance in that, besides simply containing our algorithms to be run, it specifies the parameters for each of those algorithm nodes we want to pass in. 

To add a parameter to any algorithm node for when its later launched, simply add it in its parameters section in the yaml.

## 3. The test_simulation.py file
This is where our actual test logic/test suite lives. 

\### IN DEPTH EXPLANATION FOR CRASH AND LOOP DETECTION STILL TO BE ADDED\###


## 4. The simulation_logger.py file
This is where our logging module lives.

The logging currently displays the scenario, chosen parameters (corresponding to the config/algorithms.yaml) and whether the test was a success or failure.

## Run using the following shell script

```bash
cd /racerbot_ws
mkdir -p scripts
#create a symbolic link to actual run_tests.sh
ln -s ../src/racerbot_a/test/sim_tests/run_tests.sh scripts/run_tests.sh
bash ./scripts/run_tests.sh
```



