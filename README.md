# 说明 

直接通过SDK获取图像并发布到ROS和写入到ros bag中。

# ZED图像参数
## [分辨率的选择](https://www.stereolabs.com/docs/api/python/classpyzed_1_1sl_1_1RESOLUTION.html)

| HD2K   | 2208*1242 (x2), available framerates: 15 fps.                |
| ------ | ------------------------------------------------------------ |
| HD1080 | 1920*1080 (x2), available framerates: 15, 30, 60 fps.        |
| HD1200 | 1920*1200 (x2), available framerates: 15, 30, 60 fps.        |
| HD720  | 1280*720 (x2), available framerates: 15, 30, 60 fps          |
| SVGA   | 960*600 (x2), available framerates: 15, 30, 60, 120 fps.     |
| VGA    | 672*376 (x2), available framerates: 15, 30, 60, 100 fps.     |
| AUTO   | Select the resolution compatible with camera, on ZED X HD1200, HD720 otherwise |

## [可捕获的视图](https://www.stereolabs.com/docs/api/python/classpyzed_1_1sl_1_1VIEW.html)

| HD2K                   |
| ---------------------- |
| LEFT                   |
| RIGHT                  |
| LEFT_GRAY              |
| RIGHT_GRAY             |
| LEFT_UNRECTIFIED       |
| LEFT_UNRECTIFIED_GRAY  |
| RIGHT_UNRECTIFIED_GRAY |
| SIDE_BY_SIDE           |
| DEPTH                  |
| CONFIDENCE             |
| NORMALS                |
| DEPTH_RIGHT            |
| NORMALS_RIGHT          |

## [计算获得的视图](https://www.stereolabs.com/docs/api/python/classpyzed_1_1sl_1_1MEASURE.html)

| DISPARITY          | Disparity map. Each pixel contains 1 float. sl.MAT_TYPE.F32_C1 |
| ------------------ | ------------------------------------------------------------ |
| DEPTH              | Depth map, in sl.UNIT defined in sl.InitParameters. Each pixel contains 1 float. sl.MAT_TYPE.F32_C1 |
| CONFIDENCE         | Certainty/confidence of the depth map. Each pixel contains 1 float. sl.MAT_TYPE.F32_C1 |
| XYZ                | Point cloud. Each pixel contains 4 float (X, Y, Z, not used). sl.MAT_TYPE.F32_C4 |
| XYZRGBA            | Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color  needs to be read as an unsigned char[4] representing the RGBA color.  sl.MAT_TYPE.F32_C4 |
| XYZBGRA            | Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color needs to be  read as an unsigned char[4] representing the BGRA color.  sl.MAT_TYPE.F32_C4 |
| XYZARGB            | Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color needs to be  read as an unsigned char[4] representing the ARGB color.  sl.MAT_TYPE.F32_C4 |
| XYZABGR            | Colored point cloud. Each pixel contains 4 float (X, Y, Z, color). The color needs to be  read as an unsigned char[4] representing the ABGR color.  sl.MAT_TYPE.F32_C4 |
| NORMALS            | Normals vector. Each pixel contains 4 float (X, Y, Z, 0). sl.MAT_TYPE.F32_C4 |
| DISPARITY_RIGHT    | Disparity map for right sensor. Each pixel contains 1 float. sl.MAT_TYPE.F32_C1 |
| DEPTH_RIGHT        | Depth map for right sensor. Each pixel contains 1 float. sl.MAT_TYPE.F32_C1 |
| XYZ_RIGHT          | Point cloud for right sensor. Each pixel contains 4 float (X, Y, Z, not used). sl.MAT_TYPE.F32_C4 |
| XYZRGBA_RIGHT      | Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z,  color). The color needs to be read as an unsigned char[4] representing  the RGBA color. sl.MAT_TYPE.F32_C4 |
| XYZBGRA_RIGHT      | Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z,  color). The color needs to be read as an unsigned char[4] representing  the BGRA color. sl.MAT_TYPE.F32_C4 |
| XYZARGB_RIGHT      | Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z,  color). The color needs to be read as an unsigned char[4] representing  the ARGB color. sl.MAT_TYPE.F32_C4 |
| XYZABGR_RIGHT      | Colored point cloud for right sensor. Each pixel contains 4 float (X, Y, Z,  color). The color needs to be read as an unsigned char[4] representing  the ABGR color. sl.MAT_TYPE.F32_C4 |
| NORMALS_RIGHT      | Normals vector for right view. Each pixel contains 4 float (X, Y, Z, 0). sl.MAT_TYPE.F32_C4 |
| DEPTH_U16_MM       | Depth map in millimeter whatever the sl.UNIT defined in sl.InitParameters.  Invalid values are set to 0, depth values are clamped at 65000. Each  pixel contains 1 unsigned short. sl.MAT_TYPE.U16_C1 |
| DEPTH_U16_MM_RIGHT | Depth map in millimeter for right sensor. Each pixel contains 1 unsigned short. sl.MAT_TYPE.U16_C1 |