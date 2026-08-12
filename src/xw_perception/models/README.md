# Perception models

## Required

Place Rockchip **YOLOv8n-pose** INT8 RKNN for RK3588 here:

```
yolov8n-pose.rknn
```

### Convert (on x86 PC with rknn-toolkit2)

```bash
# 1) Download ONNX from Rockchip model zoo delivery
curl -L -o yolov8n-pose.onnx \
  'https://ftrg.zbox.filez.com/v2/delivery/data/95f00b0fc900458ba134f8b180b3f7a1/examples/yolov8_pose/yolov8n-pose.onnx'

# 2) From airockchip/rknn_model_zoo examples/yolov8_pose/python:
python convert.py yolov8n-pose.onnx rk3588 i8 ./yolov8n-pose.rknn

# 3) Copy onto the robot:
scp yolov8n-pose.rknn radxa@<robot>:/home/radxa/ros2_ws/src/xw_perception/models/
```

Or run helpers:

```bash
bash /ros2_ws/src/xw_perception/models/fetch_model.sh   # download ONNX
bash /ros2_ws/src/xw_perception/models/convert_rknn.sh  # needs rknn-toolkit2
bash /ros2_ws/scripts/install_perception_deps.sh        # rknnlite in container
```

Without the `.rknn` file the perception node still starts, but publishes empty tracks /
`source=unavailable` until the model is present and `rknnlite` can init the NPU.
