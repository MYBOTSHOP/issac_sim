# Omniverse ROS

>[!IMPORTANT]
> Applicable for Ubuntu 22.04 and ROS2 Humble and Isaac Sim 4.0.0


## Installation Instruction

- [Nvidia Issac Sim Compatibility](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html)
  - [Omniverse Downloads](https://www.nvidia.com/en-us/omniverse/download/)
    - `sudo chmod +x omniverse-launcher-linux.AppImage`
    - `sudo apt update && sudo apt install fuse`
    - Double click
    - Disable IOMMU
      - `sudo bash -c 'echo GRUB_CMDLINE_LINUX="amd_iommu=off" >> /etc/default/grub'`
      - `sudo update-grub`
      - `sudo reboot`
  - Install 
    - Cache
    - Nucleus
    - Issac Sim

- Launch Issac Sim

- Add to **.bashrc** 

```bash
# Issac Sim ROS Variables
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/administrator/issac_sim/fastdds.xml
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp # Optional if required
```

- Add build

```bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

- Run directly so that the file is sourced

```bash
bash ~/.local/share/ov/pkg/isaac-sim-4.0.0/isaac-sim.sh
```

- Go to `Windows` -> `Extension Manager` -> Enable `ROS2 Bridge` & `ROS2 Robot Description URDF`
- Go to `Create` -> `Isaac` -> `Environment` -> `Flat Grid`

## ROS2 - Importing Robot 

- Go to `Issac Utils` -> `Workflows` -> `URDF Importer` 
  - Add the robot
  - Ensure **base_link** is not checked as it is a mobile robot
  - Check the drive type, usually velocity for wheel based robots
  - Self-collision disable if URDF limits are correct
  - Select **output directory**
- **Play** Animation and then **Stop**

## ROS2 - Robot Controller 

- Click `Issac Utils` -> `Common Omnigraph` -> `Articulation Position Controller` and/or `Articulation Velocity Controller`
  - Press `Add` and select the `robot`
  - Click `Ok`
  - Right Click on the `Graphs` in the Stage and select option `Open Graphs`
- Click Play and in the graphs, one of them will be a controller graph change values to see if it is reflected on robot