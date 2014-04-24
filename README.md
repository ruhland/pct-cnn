# Point Cloud Transformation via Coherent Nearest Neighbours

Point cloud transformation with coherent nearest neighbours

## Installtion on Arch Linux
* Install the [openni-git](https://aur.archlinux.org/packages/openni-git/) package from the AUR.
* Install the [pcl](https://aur.archlinux.org/packages/pcl/) with all dependencies from the AUR.
Make sure that openni-git is found.
* Install packages [primesense-nite2](https://aur.archlinux.org/packages/primesense-nite2/) 
and [sensorkinect-git](https://aur.archlinux.org/packages/sensorkinect-git/).

In order to execute the examples the kernel module gspca_kinect must be removed.
For one session this can be done with:
```
sudo modprobe -r gspca_kinect
```
To disable the module permanently, add an entry to modprobe.d:
```
$ sudo vim /etc/modprobe.d/disable_gspca_kinect
```
with the content
```
blacklist gspca_kinect
```

Resources
------
The RGB-D Object Dataset. A collection of various household object models.
Available for non-commercial reseach/educational use: http://rgbd-dataset.cs.washington.edu/dataset.html
