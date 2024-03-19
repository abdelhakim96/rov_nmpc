# mpc_acado_ros2


## Installation On Ubuntu 20.0.


Install ros2 foxy

source ros2 foxy in bashrc

Update
```
sudo apt update
sudo apt upgrade
sudo apt install git
```

In a folder of your choice, clone and install PX4 latest version using:
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo reboot
```

Install DDS (alternative of MAVROS)
```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

Clone PX4-ROS2 bridge packages in your ros2_ws and build
```
cd ~/ros2_ws/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/abdelhakim96/mpc_casadi_ros2
cd ..
colcon build
```


## Updating the Drone Model

### Steps:

1. **Update Model:**
    - Edit `code_gen_drone.cpp` in `~/mpc_acado/src/code_generation`.

2. **Build ROS2 Workspace:**
    - Run `colcon build` to incorporate changes.

3. **Generate ACADO C-Code:**
    - Execute `./code_gen_drone` to create C-code in `/mpc_acado/model/codegen`.

4. **Rebuild Workspace:**
    - Use `colcon build` again for integration.




## Running the simulation


Run PX4 SITL simulation in gazebo
```
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```

In a new terminal Run DDS
```
MicroXRCEAgent udp4 -p 8888
```

In a new terminal Run MPC
```
ros2 run mpc_acado mpc_acado_node 
```


build
```
colcon build --symlink-install
```
