pct-cnn
=======

point cloud transformation with coherent nearest neighbors

Installtion on Arch Linux
------
*Install the openni-git package from the AUR
*Install the pcl with all dependencies from the AUR. It is important that it states openni found.
*Install packages primesense-nite2 and sensorkinect-git

In order to compile the examples the kernel module gspca_kinect must be removed.
For one session this can be done with:
<pre>
sudo modprobe -r gspca_kinect
</pre>
