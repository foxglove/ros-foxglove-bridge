#!/bin/bash -e

# Here we can assert that the parameters value set by the user are proper.

# Get the parameters values.
# They may be 'unset' as per the install hook.
port="$(snapctl get port)"
address="$(snapctl get address)"
tls="$(snapctl get tls)"
certfile="$(snapctl get certfile)"
keyfile="$(snapctl get keyfile)"
topic_whitelist="$(snapctl get topic-whitelist)"
param_whitelist="$(snapctl get param-whitelist)"
service_whitelist="$(snapctl get service-whitelist)"
client_topic_whitelist="$(snapctl get client-topic-whitelist)"
min_qos_depth="$(snapctl get min-qos-depth)"
max_qos_depth="$(snapctl get max-qos-depth)"
num_threads="$(snapctl get num-threads)"
send_buffer_limit="$(snapctl get send-buffer-limit)"
use_sim_time="$(snapctl get use-sim-time)"
use_compression="$(snapctl get use-compression)"
capabilities="$(snapctl get capabilities)"
include_hidden="$(snapctl get include-hidden)"
asset_uri_allowlist="$(snapctl get asset-uri-allowlist)"

# If the parameter has been explicitely set by the user
if [ -n "${port}" ]; then
  # Check that it is a valid value
  if ! expr "${port}" : '^[0-9]\+$' > /dev/null; then
    logger -t ${SNAP_NAME} "'${opt}' is not a valid port value."
    echo "'${port}' is not a valid port value" >&2
    return 1
  fi
fi

for opt in max_qos_depth num_threads send_buffer_limit; do
  if [ -n "${!opt}" ]; then
    # Check that it is a valid value
    if ! expr "${!opt}" : '^[0-9]\+$' > /dev/null; then
      logger -t ${SNAP_NAME} "'${!opt}' is not a valid value for ${opt}."
      echo "'${!opt}' is not a valid value for ${opt}." >&2
      return 1
    fi
  fi
done

for opt in tls include_hidden use_sim_time use_compression; do
  if [ -n "${!opt}" ]; then
    case "${!opt}" in
        True) ;;
        False) ;;
        *)
        logger -t ${SNAP_NAME} "'${!opt}' is not a valid boolean for ${opt}. Please use 'True' or 'False'."
        echo "'${!opt}' is not a valid boolean for ${opt}. Please use 'True' or 'False'." >&2
        return 1
        ;;
    esac
  fi
done

# @todo assert the rest of the parameters.

