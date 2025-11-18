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
- Download yolo model from https://docs.ultralytics.com/models/yolo/
- (Optional) To use TensorRT, export the model using
  ```
  yolo export model={path_to_yolov8_pt_model} format=engine device=0
  ```
- Provide the model path in yolo.yaml config file.

## Examples

    ros2 launch waywiser_perception yolo.launch.py
    ros2 launch waywiser_perception yolo.launch.py use_sim_time:=true
    ros2 launch waywiser_perception yolo.launch.py yolo_config:=./src/WayWiseR/waywiser_perception/config/yolov8.yaml
    ros2 launch waywiser_perception yolo.launch.py yolo_config:=config_file_of_your_choice
