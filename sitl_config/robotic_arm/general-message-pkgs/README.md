# general-message-pkgs

Collection of various message packages which can be useful to a broader range of other packages.

# Dependencies

* [object_recognition_msgs](http://wiki.ros.org/object_recognition_msgs)
* This repository itself: [general-message-pkgs](https://github.com/JenniferBuehler/general-message-pkgs)

# Install

```
sudo apt-get install \
    ros-<distro>-object-recognition-msgs
```
 
Add the git repository to your catkin workspace:

```
cd <your-catkin-ws>/src
git clone https://github.com/JenniferBuehler/general-message-pkgs.git
```

*Hint*: Alternatively to cloning the repositry directly into the catkin source folder, you
may also clone the repository elsewhere and then create a softlink to the main folders
in your catkin source directory:    
``ln -s <path to general-message-pkgs>`` 
 
To compile, you can now use catkin\_make as usual:

```
cd ..
catkin_make
```
