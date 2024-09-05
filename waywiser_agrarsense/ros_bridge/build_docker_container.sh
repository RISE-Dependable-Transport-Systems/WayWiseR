#!/bin/bash

# Set default values
DOCKER_IMAGE_NAME="agrarsense-ros1-bridge:latest"
DOCKERFILE_DIR="src/WayWiseR/waywiser_agrarsense/ros_bridge"
DOCKER_CONTAINER_NAME="agrarsense_ros_bridge"

# Parse optional arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
    --image-name)
        DOCKER_IMAGE_NAME="$2"
        shift
        ;;
    --dockerfile-dir)
        DOCKERFILE_DIR="$2"
        shift
        ;;
    *)
        echo "Unknown parameter passed: $1"
        exit 1
        ;;
    esac
    shift
done

echo "Building Docker image $DOCKER_IMAGE_NAME from $DOCKERFILE_DIR..."
docker build -t "$DOCKER_IMAGE_NAME" $DOCKERFILE_DIR

# Check if the Docker container exists
if docker ps -a --format '{{.Names}}' | grep -q "^${DOCKER_CONTAINER_NAME}$"; then
    echo "Docker container $DOCKER_CONTAINER_NAME exists. Build changes won't take effect until it is removed or renamed."
    echo "To remove it, use the following command:"
    echo "docker rm $DOCKER_CONTAINER_NAME"
    echo "To rename it, use the following command (replace 'new_container_name' with the desired name):"
    echo "docker rename $DOCKER_CONTAINER_NAME new_container_name"
fi
