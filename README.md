# aida-motor
A go-to repository for ready-to-use ROS2 implementations of various motors/actuators

# Prepare the WorkSpace
To be able to use the motors/actuators in ROS2, you will need to:

```sh
mkdir ~/aida_ws && cd ~/aida_ws
```

```sh
git clone https://github.com/osinenkop/aida-motor.git -b main
```

# Clone Motor/Actuator packages
In order to download each desired motor/actuator package:

```sh
cd ~/aida_ws/aida-motor
```

and to see the list of available motors/actuators:

```sh
git branch -a
```



## COPY and MERGE the `launch` and `config` 
Now ***COPY and MERGE the `launch` and `config` folders*** to your own package. You may need to add more configuration files to the `config` directory.

***To be able to use these packages***, you will also need to have  the `launch` and `config` in your package CMakeLists.txt. If you haven't done so, should have the following:

```cmake
install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
```

## Build
```sh
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release