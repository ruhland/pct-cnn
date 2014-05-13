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
##### Installation on Ubuntu (14.04)
This setup has been tested on a Ubuntu 14.04 AMD64 VM.
Unfortunately, the installation of the libpcl-all package according to the instructions on the PCL homepage resulted in unresolved problems. Our workaround was to compile the PCL sources ourself: https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.1.tar.gz
* Install the packages:
```
cmake g++ git libboost-all-dev libflann-dev libeigen3-dev libopenni-dev  libusb-1.0 libvtk5-dev libvtk-java libvtk5-qt4-dev tcl-vtk libvtk-java python-vtk libqhull-dev libopencv-dev libopenni-sensor-primesense0
```
* Compile the sources with `mkdir build`, `cd build` and
```
cmake ../ \
-DBUILD_visualization=ON \
-DBUILD_app_cloud_composer=ON \
-DBUILD_app_in_hand_scanner=ON \
-DBUILD_app_modeler=ON \
-DBUILD_app_point_cloud_editor=ON \
```
* The actual building takes some while and needs a lot of memory: `make -j 5` if more than 5 GB RAM available. `make` otherwise
* Install with `make install`

##### Compling
Our project currently consists of several small sample demos which can be used to test various PCL components.
Running `$ cmake .` in the top-level directory will create a Makefile for every subproject.
You can compile a subproject by running `$ make` in the corresponding subdirectory.
Note: Compile times are horrible as of right now, because of the excessive C++ templating. We're currently looking for ways to resolve this issue.

### Resources
The RGB-D Object Dataset. A collection of various household object models.
Available for non-commercial reseach/educational use: http://rgbd-dataset.cs.washington.edu/dataset.html
