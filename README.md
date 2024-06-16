## hikrobot_mvs_ros package

This is a ros package for hikrobot mvs camera.

### support
1. Support both IP and USB3 camera. If your computer connects only single camera, this package will search the camera automaticly. If multi cameras are connected, please modify `SerialNumber` param in `camera.yaml`.
2. Support modify camera size.
   1. If you want crop the image, please modify `Width` and `Height` params in `camera.yaml`.
   2. If you want downsample the image, please modify `SubSample` param (the down sample scale) in `camera.yaml`.
   3. Note: Increase the camera frequency by compress the image size.
3. Support hard synchronization with LiDAR, which can be used by LIVO system.