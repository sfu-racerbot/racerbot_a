#!/usr/bin/env bash
set -eo pipefail

# Find dir location of this script
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

# Use that location to find workspace root
WS_ROOT="$(cd "$SCRIPT_DIR/../../../../" && pwd)"

cd "$WS_ROOT"

# Source the workspace 
source $WS_ROOT/.venv/bin/activate
source /opt/ros/humble/setup.bash

colcon build --packages-select sim_tests

#build algorithm nodes
colcon build --packages-select \
    ftg \
    disparity_extender \
    pure_pursuit_a \
    reactive \

#build safety node 
colcon build --packages-select safety_node
    
source install/local_setup.bash

MAP_CONFIG="$WS_ROOT/src/f1tenth_gym_ros/config"
MAPS_DIR="$WS_ROOT/src/f1tenth_gym_ros/maps"

MAPS=("levine_blocked" "levine_obs" "Spielberg")

#ensure that the the 3 arrays always match up lengthwise and orderwise
ALGORITHM_PACKAGES=("ftg" "disparity_extender")
ALGORITHM_EXECUTABLES=("ftg_node" "disparity_extender_node")
# "min speed, max_speed"
ALGORITHM_TEST_SPEEDS=(
    "2.0,5.5" 
    "0.7,4.5" 
)

# Select map (currently only works with levine)
export SIM_CONFIG="$SCRIPT_DIR/config/sim.yaml"


for map in "${MAPS[@]}"; do
    #Select map 
    #Using stream editor to replace the whole line
    #for e.g #thing to be replaced#replacement#
    export MAP=${map}
    sed -i "s#map_path: '.*'#map_path: '${MAPS_DIR}/${map}'#" $SIM_CONFIG
    for ((i=0; i<${#ALGORITHM_EXECUTABLES[@]}; i++)); do
        #individual field seperator used to extract min and max speeds
        IFS=',' read -r MIN_SPEED MAX_SPEED <<< "${ALGORITHM_TEST_SPEEDS[i]}"
        export MIN_SPEED
        export MAX_SPEED

        export ALGORITHM_PACKAGE=${ALGORITHM_PACKAGES[i]}
        export ALGORITHM_EXECUTABLE=${ALGORITHM_EXECUTABLES[i]}

        export PYTEST_ADDOPTS="--capture=no"
        #run tests
        colcon test --packages-select sim_tests \
            --return-code-on-test-failure \
            --event-handlers console_direct+
        colcon test-result --verbose

        TEST_RESULTS_DIR="$WS_ROOT/build/sim_tests/test_results/sim_tests"
        mkdir -p "$WS_ROOT/results"
        cp "$TEST_RESULTS_DIR/simulation.xunit.xml" "$WS_ROOT/results/${map}_${ALGORITHM_PACKAGES[i]}.xunit.xml" 
    done
done



