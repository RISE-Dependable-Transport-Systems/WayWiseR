# waywiser_perception

## YOLO Model Setup

- Download yolo model from https://docs.ultralytics.com/models/
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
