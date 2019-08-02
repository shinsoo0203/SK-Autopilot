SK-Autopilot
===================

install gazebo
-------------------
```
$ sudo apt-get update  
$ sudo apt-get install gazebo7  
$ sudo apt-get install libgazebo7-dev  
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs  
```

install mavros
--------------------
```
$ sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras  
$ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh  
$ ./install_geographiclib_datasets.sh  
```

dependancies
--------------------
```
$ sudo apt-get install python-catkin-tools python-rosinstall-generator -y  
```

install PX4
--------------------
```
$ sudo add-apt-repository ppa:george-edison55/cmake-3.x -y  
$ sudo apt-get update  
$ sudo apt-get install python-argparse git-core wget zip python-empy qtcreator cmake build-essential genromfs -y  
$ sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-8-jdk openjdk-8-jre clang-3.5 lldb-3.5 -y  
$ mkdir -p ~/src  
$ cd ~/src  
$ git clone https://github.com/PX4/Firmware.git  
$ cd Firmware  
$ git submodule update --init --recursive  
$ make px4_sitl_default gazebo  
```

build SK-Autopilot
---------------------
```
$ cd SK-Autopilot  
$ catkin build  
```
