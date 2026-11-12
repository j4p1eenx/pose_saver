# Usage: source scripts/docker_helpers.sh   (from the repo root)
SETUP="source /ros2_ws/install/setup.bash"

# Start the whole demo. Extra args are launch arguments, e.g.: dlaunch x_pose:=-2.0 y_pose:=-0.5 yaw_pose:=0.0
dlaunch() { docker compose run --rm --name sim demo bash -c "$SETUP && exec ros2 launch pose_saver_pkg pose_saver_turtlebot.launch.py $*"; }

# CRUD against the database: dcli create|read|readall|update|delete ...
dcli() { docker compose run --rm demo bash -c "$SETUP && ros2 run pose_saver_pkg pose_crud_cli \$POSE_DB_URL $*"; }

# Attach to the running sim (start it first with dlaunch)
dstatus() { docker exec -it sim bash -c "$SETUP && ros2 topic echo /pose_saver_node/status --qos-durability transient_local --qos-reliability reliable"; }
dteleop() { docker exec -it sim bash -c "$SETUP && export TURTLEBOT3_MODEL=burger && ros2 run turtlebot3_teleop teleop_keyboard"; }

# Unit tests inside the image
dtest() { docker compose run --rm demo bash -c "$SETUP && cd /ros2_ws && colcon test --packages-select pose_saver_pkg --event-handlers console_direct+ && colcon test-result --verbose"; }

# Housekeeping
dclean() { docker rm -f sim >/dev/null 2>&1; echo "stale sim container removed"; }
dreset() { docker compose down -v; echo "cache volume wiped"; }
