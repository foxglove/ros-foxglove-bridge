#!/usr/bin/sh -e

LAUNCH_OPTIONS=""

OPTIONS="port
address
tls
certfile
keyfile
topic-whitelist
param-whitelist
service-whitelist
client-topic-whitelist
min-qos-depth
max-qos-depth
num-threads
send-buffer-limit
use-sim-time
use-compression
capabilities
include-hidden
asset-uri-allowlist"

for OPTION in ${OPTIONS}; do
  VALUE="$(snapctl get ${OPTION})"
  if [ -n "${VALUE}" ]; then
    LAUNCH_OPTIONS="${LAUNCH_OPTIONS} ${OPTION}:=${VALUE}"
  fi
done

# Replace '-' with '_'
LAUNCH_OPTIONS=$(echo ${LAUNCH_OPTIONS} | tr - _)

if [ "${LAUNCH_OPTIONS}" ]; then
  logger -t ${SNAP_NAME} "Running with options: ${LAUNCH_OPTIONS}"
fi

ros2 launch foxglove_bridge foxglove_bridge_launch.xml ${LAUNCH_OPTIONS}
