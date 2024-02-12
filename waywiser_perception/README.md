## Requirements

### Jetson

- ```
  sudo apt-get install python3-pip libopenblas-base libopenmpi-dev libomp-dev python3-libnvinfer-dev libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
  ```
- Install torch and torchvision following the instructions from https://forums.developer.nvidia.com/t/pytorch-for-jetson/
- Install pip dependencies:
  ```
  pip install numpy==1.23 ultralytics opencv-python pycuda
  ```

### x86_64

- To use Nvidia GPUs for inference,

  - install cuda toolkit if not already installed (https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#ubuntu).
  - ```
    pip install pycuda
    ```

- Install pip dependencies:

  ```
  pip install numpy ultralytics opencv-python torch torchvision
  ```

## Setup

- ```
  rosdep install --from-paths src/WayWiseR/waywiser_perception -y -r --ignore-src
  ```
- Download yolov8 model from https://docs.ultralytics.com/models/yolov8/
- (Optional) To use TensorRT, export the model using
  ```
  yolo export model={path_to_yolov8_pt_model} format=engine device=0
  ```
- Provide the model path in yolov8.yaml config file.

## Examples

    ros2 launch waywiser_perception yolov8.launch.py
    ros2 launch waywiser_perception yolov8.launch.py use_sim_time:=false
    ros2 launch waywiser_perception yolov8.launch.py yolov8_config:=./src/WayWiseR/waywiser_perception/config/yolov8.yaml
    ros2 launch waywiser_perception yolov8.launch.py yolov8_config:=config_file_of_your_choice

    ros2 launch waywiser_perception realsense_xyzrgb_pointcloud.launch.py
    ros2 launch waywiser_perception realsense_xyzrgb_pointcloud.launch.py use_sim_time:=false
    ros2 launch waywiser_perception realsense_xyzrgb_pointcloud.launch.py container:=/sensors/camera/camera_container
    ros2 launch waywiser_perception realsense_xyzrgb_pointcloud.launch.py namespace:=/sensors/drone/camera
