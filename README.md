# irsl_robot_hardware_dynamixel

## build

```
sudo apt install python-vcstool
mkdir -p <your_work_space>/src
source /opt/ros/<distro>/setup.bash
cd <your_work_space>
wget https://raw.githubusercontent.com/IRSL-tut/irsl_ros_control_shm/refs/heads/release-candidate/test/install.vcs
(cd /src; vcs import --recursive < ../intall.vcs)
catkin init
catkin config --install
catkin build irsl_dynamixel_hardware_shm irsl_ros_control_shm
```

## Execute

### example

- run hardware

```
source <your_work_space>/install/setup.bash
dynamixel_hardware_shm --hash 8888 --shm_key 8888 --config <path_to>/config.yaml
```

- run control

```
source <your_work_space>/install/setup.bash
cd <your_work_space>/src/irsl_ros_control_shm/test
roslaunch ./ros_control_dynamixel.launch hash:=8888 shm_key:=8888
```

# Old

### Prepare related librarys
```
cd /
git clone https://github.com/IRSL-tut/irsl_shm_controller_library -b release-candidate

cd /irsl_shm_controller_library/irsl_shm_libs
mkdir -p build
cd build
cmake .. && make -j `nproc` && make install

cd /irsl_shm_controller_library/irsl_common_utils
mkdir -p build
cd build
cmake .. && make -j `nproc` && make install

cd /irsl_shm_controller_library/irsl_realtime_utils
mkdir -p build
cd build
cmake .. && make -j `nproc` && make install

ldconfig

```

```
cd <ros_workspace>/src
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git -b noetic
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b noetic
cd ..
catkin build dynamixel_sdk dynamixel_workbench_toolbox
```

<!-- 
shm側を入れる
cd <ros_workspace>/src

-->

### clone and build
```
cd <source dir>
git clone https://github.com/xxx/irsl_robot_hardware_dynamixel.git
cd irsl_robot_hardware_dynamixel
mkdir build
cd build
cmake ..
make 
```

## Execute 
### example
```
cd <source dir>/irsl_robot_hardware_dynamixel/build
./robot_control 8888 88888 config.yaml
```

### Command line Option
| Option Name     | Type / Example                                       | Description                                                       | Default Value                     |
| --------------- | ---------------------------------------------------- | ----------------------------------------------------------------- | --------------------------------- |
| `shm_hash`      | String<br>Example: `1234`                            | Specifies the shared memory hash value.                           | `"8888"`                          |
| `shm_key`       | String<br>Example: `5678`                            | Specifies the shared memory key.                                  | `"8888"`                          |
| `config_file`   | String<br>Example: `config.yaml`                     | Specifies the name of the input configuration file (YAML format). | `"config.yaml"`                   |
| `--joint_type`  | String<br>Example: `"PositionGains,PositionCommand"` | Specifies the joint types (comma-separated list).                 | `"PositionGains,PositionCommand"` |
| `-v, --verbose` | Flag                                                 | Enables verbose output.                                           | *(Default: Off)*                  |

#### Valid Values for `--joint_type`
| Joint Type Name    |
| ------------------ |
| `PositionCommand`  |
| `PositionGains`    |
| `VelocityCommand`  |
| `VelocityGains`    |
| `TorqueCommand`    |
| `TorqueGains`      |
| `MotorTemperature` |
| `MotorCurrent`     |
