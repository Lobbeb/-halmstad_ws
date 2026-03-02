source /opt/ros/jazzy/setup.bash

# Resolve this workspace dynamically so setup works across user accounts.
_CLEARPATH_SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_WS_ROOT="$(cd "${_CLEARPATH_SETUP_DIR}/../../.." && pwd)"
if [ -f "${_WS_ROOT}/install/setup.bash" ]; then
  source "${_WS_ROOT}/install/setup.bash"
fi
unset _CLEARPATH_SETUP_DIR
unset _WS_ROOT

export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
