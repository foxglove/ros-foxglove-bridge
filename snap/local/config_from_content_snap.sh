#!/bin/bash -e

CONFIGURATION_FILE_PATH=$SNAP_COMMON/configuration/foxglove-bridge.yaml

if [ ! -f "$CONFIGURATION_FILE_PATH" ]; then
    echo "Configuration file '$CONFIGURATION_FILE_PATH' does not exist."
    exit 1
fi

snapctl set port=$(yq '.port // 54321' $CONFIGURATION_FILE_PATH)
snapctl set address=$(yq '.address // "0.0.0.0"' $CONFIGURATION_FILE_PATH)
snapctl set topic-whitelist=$(yq '.topic-whitelist // "*"' $CONFIGURATION_FILE_PATH)

$SNAP/usr/bin/validate_config.sh

