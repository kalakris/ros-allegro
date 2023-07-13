# Deployment of Allegro Hand

## Disclaimer

This repository and README is heavily based on Haozhi's Allegro ROS repository (https://github.com/HaozhiQi/ros-allegro.git)

## Installation

### CAN Setup

#### Socket CAN
  [Optional] disable previously installed peakcan driver
  ```
  # Temporarily
  sudo rmmod pcan
  sudo modprobe peak_usb

  # Permanent
  rm /etc/modprobe.d/pcan.conf
  rm /etc/modprobe.d/blacklist-peak.conf
  ```

  Set bitrate
  ```
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set can0 up
  ```


#### PCAN (not needed for SocketCAN User)

    Download the [peak linux driver](http://www.peak-system.com/fileadmin/media/linux/index.htm#download) and unzip it. In the driver folder, execute the following command:
    ```
    # in peak-linux-driver
    make clean
    make NET=NO_NETDEV_SUPPORT
    sudo make install
    sudo /sbin/modprobe pcan
    ```
    After this above installation, you should be able to see `/dev/pcanusb*` in your machine.


### Install this repository

```
cd ${CATKIN_WS}/src/
git clone https://github.com/0wu/ros-allegro.git
cd ../
catkin_make -j1 -DCANDRV=SOCKETCAN
# catkin_make -DCANDRV=PEAKCAN #for peakcan driver
catkin_make install
source ./devel/setup.zsh # or other shell name
```

### Instructions

Can use `rostopic list` to monitor the ROS topics.

Try a communication with hand
```
# need to change /dev/pcanusbfd32 to your usb name
roslaunch allegro_hand allegro_hand.launch HAND:=right AUTO_CAN:=false CAN_DEVICE:=can0 KEYBOARD:=false CONTROLLER:=pd
```

Make sure `rostopic list` will display something like:
```shell
/allegroHand_0/envelop_torque
/allegroHand_0/joint_cmd
/allegroHand_0/joint_states
/allegroHand_0/lib_cmd
```
