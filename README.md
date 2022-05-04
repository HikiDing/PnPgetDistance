# pnpGetDistance

$ bash start.bash

其中多了一个测试的find_rect方便使用usb相机进行调试

在使用solvedPNP函数的时候，自己忘记将points3D和points2D两个vector及时清空
于是导致程序abort。

还未完成的部分：
    将相机的旋转角度求出，并且测试的时候还是存在不足,例如：测距的误差


ros2中的示例pkg ：paramtest
    使用之前学习ros2的时候 设定参数的代码，改装。
    输入的point是固定值，可以通过getdistance来修改
    
    $source install/setup.bash(setup.zsh)
    $ros2 launch paramtest paramter.launch.py


相机标定：
    使用的是ros自带的包：ros-noetic-camera_calibration
    