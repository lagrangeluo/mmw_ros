### Setup CAN-To-USB adapter 

```
# enable kernel module: gs_usb
$ sudo modprobe gs_usb

# install can utils
$ sudo apt install -y can-utils
```

### Launch ROS nodes


* Start the base node for mmw_ros

    ```
    # bring up can interface
    $ sudo ip link set can0 up type can bitrate 500000
    
    $ roslaunch mmw_bringup mmw_ros.launch
    ```


* Start the sub node for mmw_status

  ```
  $ rostopic echo /mmw_status
  ```

  

