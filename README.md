# NSL-1110AA ROS2
--- NSL-1110AA ROS2 demo ---

1. Build env
 - Ubuntu22.04.1 LTS
 - ROS2 Humble
 - OPENCV 4.5.4
 
 
2. Build NSL-1110AA demo
```
$ cd NSL1110_driver
$ colcon build --packages-select roboscan_nsl1110
$ . install/setup.bash
```
 
3. Start commands
```
$ ros2 run roboscan_nsl1110 roboscan_publish_node
$ ros2 launch roboscan_nsl1110 camera.Launch.py
```

# NSL-1110AA View


  ![ROS2](https://user-images.githubusercontent.com/106071093/218378867-6792ac7b-4b2f-4227-9fa3-ef833f0fc784.png)


# Set parameters
```
$ rqt
 (reconfigure)
```

![Screenshot from 2023-02-22 13-00-21 (copy)](https://user-images.githubusercontent.com/106071093/220520356-3d16736f-902c-4d9e-858d-a6ed8ddf87aa.png)


```

cvShow : Image Viewer on/off

imageType 0 : Grayscale 
imageType 1 : Distance 
imageType 2 : Distance / Amplitude
imageType 3 : DCS

int0 , int2 = 0 ~ 4000(LED)

intGr = 0 ~ 40000(LED)

modIndex 0 : 24MHz
modIndex 1 : 12MHz
modIndex 2 : 6MHz
modIndex 3 : 3MHz


transformAngle : angle (rviz-based y-axis rotation)
```

