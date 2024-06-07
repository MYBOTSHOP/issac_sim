# Omniverse ROS

>[!IMPORTANT]
> Applicable for Ubuntu 22.04 and ROS2 Humble 


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

- Add to **.bashrc** `export ROS_DISTRO=humble`