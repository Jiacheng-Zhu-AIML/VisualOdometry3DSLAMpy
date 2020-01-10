# NAVA568_MobileRobotics_Team12

# https://gitlab.eecs.umich.edu/jiachzhu/NAVA568_MobileRobotics_Team12

There are two main parts in our code. 

1. Visual SLAM for camera pose tracking (excluding the curve-based method)
2. Lane detection and (local/global) 3D estimation (including curve-based camera pose estimation)

The first part is in "VisualSLam3D" folder, and the second part is in folder "lane_processing".

--------------------
The videos showing results for lane processing: 
Global 3D Road Reconstruction in z-x View: https://youtu.be/nIxLLVMyMIY

Global 3D Road Reconstruction in z-y View: https://youtu.be/nxGlNWxjPfo

Local 3D Road Reconstruction in z-x View: https://youtu.be/avQn5NFmLIw

Local 3D Road Reconstruction in z-y View: https://youtu.be/34YiT55xcK8

Raw observation of 3D lane from Single Monocular Image: https://youtu.be/i1Juo-08bPU


---------------------
Usage of VisualSLAM3D:

Dependency:
opencv
numpy

1. Input video/images
The algorithm uses discrete image lists as input.
To get the image list, please download the "imagelist.zip" from our group files.
Unzip the imagelist.zip file， change the path in
voslam_trajectory.py and voslam_feature_reconst.py respectively.


2. Recover the 3D camera poses/trajectory

run voslam_trajectory.py
The R (3x3) matrix and T(3x1) matrix will be printed for each frame
The bird eye view of vehicle trajectory will be plotted.

3. Recover the 3D camera poses as well as the 3D feature/landmark points

run voslam_feature_reconst.py
The bird eye view of vehicle trajectory will be plotted and the 3D feature
points will also plotted in yellow points.
A 3D plot will show all the 3D features under global coordinate.
This algorithm is not efficiency, we suggest only process 200 frames.
