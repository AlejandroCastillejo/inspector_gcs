# inspector_gcs

ROS-based software for PV plants inspections. The project is divided into ground and aerial segments.

This is the ground segment, to be installed on the ground control station.

You can find the aerial segment package [here](http://olaf.grupotsk.com:8080/inspector/us/inspector_software_uav)


## 

## How to build and install inspector_gcs 

### ROS 

This package is based on ROS Kinetic Kame.

See instructions and install "Desktop-Full" package: 
http://wiki.ros.org/kinetic/Installation/Ubuntu


### Dependencies

Before the current package, some dependencies need to be installed.
 
 * ROS-multimaster-fkie:
    ```
    sudo apt-get install ros-kinetic-multimaster-fkie 
    ```

 * Qt5 dependencies: 
    ```
	sudo apt-get install qtbase5-dev
	sudo apt-get install qtpositioning5-dev
    ```

 * sshpass (required for file transfer via scp)
    ```
    sudo apt-get install sshpass
    ```

 * python-pandas (required for data bases)
   ```
   sudo pip install pandas
   ```

### inspector_gcs package
To conclude, clone this repository into your catkin workspace and compile:
```
cd ~/(your catkin_ws)/src
git clone (repository URL)
cd ~/(your catkin_ws)
catkin_make
```