# fuel_gazebo_ws

## Ubuntu 20.04 安装 FUEL

FUEL planner 官方源码（https://github.com/HKUST-Aerial-Robotics/FUEL.git）只在Ubuntu 16.04(ROS Kinetic) and 18.04(ROS Melodic)进行了测试，在Ubuntu 20.04(ROS Noetic)上可能由于PCL版本问题无法编译成功，因此在Ubuntu 20.04(ROS Noetic)上选择重构的FUEL planner（https://gitee.com/tiemuhua/fuel_refactored）

需要以二进制的形式安装nlopt库，采用`sudo apt install ros-noetic-nlopt`命令安装缺少nlopt.so，因此会报错，因此采用以二进制的形式安装nlopt库

```bash
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install
```

添加其他依赖库

```bash
sudo apt install libdw-dev libarmadillo-dev ros-noetic-nlopt
```
