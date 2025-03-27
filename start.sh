IMAGE_NAME="ironoa/mb-ros-noetic"
CONTAINER_NAME="mb-ros-noetic"

CURRENT_DIR=$(dirname "$0")
cd $CURRENT_DIR 

docker build -t ${IMAGE_NAME}:latest .

xhost +localhost

# Check if running Docker Desktop
if docker info | grep -q "Operating System: Docker Desktop"; then
    # Docker Desktop environment (macOS or Linux with Docker Desktop)
    DISPLAY_ENV="DISPLAY=host.docker.internal:0"
else
    # Standard Linux environment
    DISPLAY_ENV="DISPLAY=$DISPLAY"
fi

docker run  -it \
            --env="${DISPLAY_ENV}" \
            --env="QT_X11_NO_MITSHM=1" \
            -v "/dev/shm:/dev/shm" \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v $(pwd)/catkin_ws:/catkin_ws \
            -v $(pwd)/docker-scripts:/docker-scripts \
            --name ${CONTAINER_NAME} \
            --rm \
            ${IMAGE_NAME}:latest \
            /docker-scripts/entrypoint.sh