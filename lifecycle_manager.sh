#!/bin/bash
set -e  # Exit immediately if a command fails

function wait_for_success() {
    local node=$1
    local transition=$2
    
    echo "Transitioning $node to $transition..."
    
    while true; do
        output=$(ros2 lifecycle set $node $transition 2>&1)
        echo "$output"
        if echo "$output" | grep -q "Transitioning successful"; then
            break
        fi
        sleep 1
    done
}

wait_for_success /map_server configure
wait_for_success /map_server activate
wait_for_success /amcl configure
wait_for_success /amcl activate

echo "✅ All lifecycle transitions completed successfully!"

