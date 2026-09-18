#!/bin/bash
set -e
SEED=/usr/src/node-red/node_modules/node-red-contrib-ros2-suite/examples/turtlesim.json
if [ ! -f "/data/${FLOWS}" ] && [ -f "$SEED" ]; then
    cp "$SEED" "/data/${FLOWS}"
fi
exec /usr/src/node-red/entrypoint.sh "$@"
