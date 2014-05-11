#### Point Cloud Transformation with Coherent Nearest Neighbours

Point cloud transformation with coherent nearest neighbours.

##### Installation on Arch Linux
* Install the [openni-git](https://aur.archlinux.org/packages/openni-git/) package from the AUR.
* Install the [pcl](https://aur.archlinux.org/packages/pcl/) with all dependencies from the AUR.
Make sure that openni-git is found.
* Install packages [primesense-nite2](https://aur.archlinux.org/packages/primesense-nite2/) 
and [sensorkinect-git](https://aur.archlinux.org/packages/sensorkinect-git/).

In order to execute the examples, the kernel module gspca_kinect must be removed.
The module can be disabled for one session with:
```
$ sudo modprobe -r gspca_kinect
```
To disable the module permanently, add an entry to modprobe.d:
```
$ sudo vim /etc/modprobe.d/disable_gspca_kinect
```
with the content
```
blacklist gspca_kinect
```
##### Installation on Ubuntu
This setup has been tested on a Ubuntu 14.04 AMD64 VM.
* Install the pcl packages according to these instruction: http://pointclouds.org/downloads/linux.html
* Install the package [libopencv-dev](http://packages.ubuntu.com/en/trusty/libopencv-dev).
* We're using Boost program options. Install [libboost-program-options-dev](http://packages.ubuntu.com/en/trusty/libboost-program-options-dev)

##### Compling
Our project currently consists of several small sample demos which can be used to test various PCL components.
Running `$ cmake .` in the top-level directory will create a Makefile for every subproject.
You can compile a subproject by running `$ make` in the corresponding subdirectory.
Note: Compile times are horrible as of right now, because of the excessive C++ templating. We're currently looking for ways to resolve this issue.

### Resources
The RGB-D Object Dataset. A collection of various household object models.
Available for non-commercial reseach/educational use: http://rgbd-dataset.cs.washington.edu/dataset.html
