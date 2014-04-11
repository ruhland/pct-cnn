# Point Cloud Transformation via Coherent Nearest Neighbours

point cloud transformation with coherent nearest neighbors

Installtion on Arch Linux
------
* Install the openni-git package from the AUR
* Install the pcl with all dependencies from the AUR. It is important that it states openni found.
* Install packages primesense-nite2 and sensorkinect-git

In order to execute the examples the kernel module gspca_kinect must be removed.
For one session this can be done with:
<pre>
sudo modprobe -r gspca_kinect
</pre>

Resources
------
The RGB-D Object Dataset. A collection of various household object models.
Available for non-commercial reseach/educational use: http://rgbd-dataset.cs.washington.edu/dataset.html
