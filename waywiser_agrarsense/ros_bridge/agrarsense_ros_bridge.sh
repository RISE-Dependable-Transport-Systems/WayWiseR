#!/bin/bash

# Defaults
SESSION_NAME_UE5_ROS1="ue5_ros1_bridge"
SESSION_NAME_ROS1_ROS2="ros1_ros2_bridge"
DOCKER_CONTAINER_NAME="agrarsense_ros_bridge"
DOCKER_IMAGE_NAME="agrarsense-ros1-bridge:latest"
HOST_LOG_DIR="$HOME/.waywiser/logs/agrarsense_ros_bridge"
ROS_PORT=9090
SLEEP_TIME=2

# Parse optional arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
    --image-name)
        DOCKER_IMAGE_NAME="$2"
        shift
        ;;
    --container-name)
        DOCKER_CONTAINER_NAME="$2"
        shift
        ;;
    --ros-port)
        ROS_PORT="$2"
        shift
        ;;
    *)
        echo "Unknown parameter passed: $1"
        echo "Usage: $0 [--image-name DOCKER_IMAGE_NAME] [--container-name DOCKER_CONTAINER_NAME] [--ros-port ROS_PORT]"
        exit 1
        ;;
    esac
    shift
done

# Ensure the host log directory exists
mkdir -p "$HOST_LOG_DIR"

# Define log file names with timestamp suffix
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
UE5_ROS1_LOG_FILE="$HOST_LOG_DIR/ue5_ros1_$TIMESTAMP.log"
ROS1_ROS2_LOG_FILE="$HOST_LOG_DIR/ros1_ros2_$TIMESTAMP.log"

CLEANUP_DONE=false

# Set up a trap to clean up
cleanup() {
    if [ "$CLEANUP_DONE" = false ]; then
        CLEANUP_DONE=true
        echo -e "\nExiting..."

        # Stop host screen sessions
        screen -S $SESSION_NAME_UE5_ROS1 -X quit &>/dev/null || true
        screen -S $SESSION_NAME_ROS1_ROS2 -X quit &>/dev/null || true

        # Stop Docker container
        docker stop --time 2 $DOCKER_CONTAINER_NAME &>/dev/null

        echo "Stopped docker $DOCKER_CONTAINER_NAME and cleaned up screen sessions."
        exit 0
    fi
}
trap cleanup SIGINT SIGTERM EXIT

# Start or reuse the Docker container
if docker ps -a --filter "name=$DOCKER_CONTAINER_NAME" | grep -w "$DOCKER_CONTAINER_NAME" &>/dev/null; then
    echo "Docker container $DOCKER_CONTAINER_NAME already exists. Starting it..."
    docker start $DOCKER_CONTAINER_NAME &>/dev/null
else
    echo "Starting new Docker container $DOCKER_CONTAINER_NAME..."
    docker run -d \
        --name $DOCKER_CONTAINER_NAME \
        --network host \
        -e ROS_PORT=$ROS_PORT \
        $DOCKER_IMAGE_NAME \
        tail -f /dev/null &>/dev/null
fi

# Start UE5<->ROS1 bridge inside a host screen session
screen -L -Logfile "$UE5_ROS1_LOG_FILE" -dmS $SESSION_NAME_UE5_ROS1 \
    docker exec -it $DOCKER_CONTAINER_NAME bash -c "\
        export ROS_PORT=${ROS_PORT} && ./start_ue5_ros1_bridge"
echo "UE5<->ROS1 bridge started using ROS_PORT=$ROS_PORT!"

sleep $SLEEP_TIME

# Start ROS1<->ROS2 bridge inside a host screen session
screen -L -Logfile "$ROS1_ROS2_LOG_FILE" -dmS $SESSION_NAME_ROS1_ROS2 \
    docker exec -it $DOCKER_CONTAINER_NAME bash -c "\
        export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && ./start_ros1_ros2_bridge"
echo "ROS1<->ROS2 bridge started using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}!"

echo "UE5<->ROS1 and ROS1<->ROS2 bridge logs are being saved in $HOST_LOG_DIR"

# Monitor logs
tail -f "$UE5_ROS1_LOG_FILE" | sed "s/^/[UE5<->ROS1 bridge] /" &
tail -f "$ROS1_ROS2_LOG_FILE" | sed "s/^/[ROS1<->ROS2 bridge] /" &

wait
