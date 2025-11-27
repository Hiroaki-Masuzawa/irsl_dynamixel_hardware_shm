# irsl_dynamixel_hardware_shm

## build

```
sudo apt install python-vcstool
mkdir -p <your_work_space>/src
source /opt/ros/<distro>/setup.bash
cd <your_work_space>
wget https://raw.githubusercontent.com/IRSL-tut/irsl_ros_control_shm/refs/heads/release-candidate/test/install.vcs
(cd src; vcs import --recursive < ../install.vcs)
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
roslaunch ros_control_shm.launch hash:=8888 shm_key:=8888 control_config:=ros_control.yaml robot_name:=dynamixel
```

### Command line Option
| Option Name     | Type / Example                                       | Description                                                       | Default Value                     |
| --------------- | ---------------------------------------------------- | ----------------------------------------------------------------- | --------------------------------- |
| `shm_hash`      | String<br>Example: `1234`                            | Specifies the shared memory hash value.                           | `"8888"`                          |
| `shm_key`       | String<br>Example: `5678`                            | Specifies the shared memory key.                                  | `"8888"`                          |
| `config_file`   | String<br>Example: `config.yaml`                     | Specifies the name of the input configuration file (YAML format). | `"config.yaml"`                   |
| `-v, --verbose` | Flag                                                 | Enables verbose output.                                           | *(Default: Off)*                  |
