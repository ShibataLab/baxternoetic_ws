# baxternoetic_ws
This code is a modification of the following repository and is being tested on ROS Noetic. See LICENSE.txt for license information.

[BaxterSDK](https://github.com/RethinkRobotics/baxter)

[baxter_common](https://github.com/RethinkRobotics/baxter_common)

[baxter_interface](https://github.com/RethinkRobotics/baxter_interface)

[baxter_tools](https://github.com/RethinkRobotics/baxter_tools)

[baxter_examples](https://github.com/RethinkRobotics/baxter_examples)

※Please note that it is not an official repository and we cannot accept any responsibility.

# System Requirements
### We have confirmed that it works with cuda 12.0.

```
nvcc -V
```
```
nvcc: NVIDIA (R) Cuda compiler driver”
Copyright (c) 2005-2023 NVIDIA Corporation
Built on Fri_Jan__6_16:45:21_PST_2023
Cuda compilation tools, release 12.0, V12.0.140
＃Build cuda_12.0.r12.0/compiler.32267302_0
```
```
nvidia -smi
```
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 525.85.12    Driver Version: 525.85.12    CUDA Version: 12.0     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  On   | 00000000:01:00.0  On |                  N/A |
| 36%   34C    P8    11W / 170W |    310MiB / 12288MiB |      0%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+

```

```
nvidia-container-cli info
```
```
NVRM version:   525.85.12
CUDA version:   12.0

Device Index:   0
Device Minor:   0
Model:          NVIDIA GeForce RTX 3060
Brand:          GeForce
GPU UUID:       GPU-901fa2c2-87a8-7307-1ab0-b3ab4e023cf8
Bus Location:   00000000:01:00.0
Architecture:   8.6

```


## Docker Setting
Please install the NVIDIA Container Toolkit beforehand. This will allow you to use gazebo, etc.

### pull docker enviroment
```
docker pull kakeru58/baxter_noetic_cuda12.0:latest
```
# How to use
## Download this repository
```
git clone https://github.com/ShibataLab/baxternoetic_ws.git
```
## Enter Docker Env
```
cd baxternoetic_ws
./launch.sh
```
Now a new terminal should open.

---From here you can work in a new terminal, within the Docker environment.---
```
catkin build
```
Edit baxter.sh to your environment
```
./baxter.sh
```
If you want to use simulation

```
./baxter.sh sim
roslaunch baxter_gazebo baxter_world.launch
```



## Enale robot
```
rosrun baxter_tools enable_robot.py -e
```

## Test
```
rosrun baxter_examples joint_velocity_wobbler.py
```

# Contact & Support us
This repository is incomplete and may cause errors, especially with existing python code.

やPlease let us know if you find an error in an issue, or in a pull request if you resolve it.

# Copyright
Copyright (c) 2013-2015, Rethink Robotics
All rights reserved.

