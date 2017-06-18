# LSD-SLAM: Large-Scale Direct Monocular SLAM
**This Version Don't need ROS**

LSD-SLAM is a novel approach to real-time monocular SLAM. It is fully direct (i.e. does not use keypoints / features) and creates large-scale, 
semi-dense maps in real-time on a laptop. For more information see
[http://vision.in.tum.de/lsdslam](http://vision.in.tum.de/lsdslam)
where you can also find the corresponding publications and Youtube videos, as well as some 
example-input datasets, and the generated output as rosbag or .ply point cloud.

### Related Papers

* **LSD-SLAM: Large-Scale Direct Monocular SLAM**, *J. Engel, T. Schöps, D. Cremers*, ECCV '14

* **Semi-Dense Visual Odometry for a Monocular Camera**, *J. Engel, J. Sturm, D. Cremers*, ICCV '13

# I want to thank
aivijay/lsd_slam_noros
I take this repo and fix some bugs to make it working with linux and testing using GCC 5.4 and CLang 4

# How to build from source
The lsd slam code requires c++11 features.
Thus, it needs c++11 supported compiler to build the code from source.

# Required Packges
    sudo apt install build-essential git cmake ninja
    sudo apt install libboost-all-dev libeigen3-dev libsuitesparse-dev libopencv-dev
    # Install g2o
    git clone https://github.com/RainerKuemmerle/g2o.git
    cd g2o
    mkdir build
    cd build
    cmake -G Ninja ..
    sudo ninja install

# Building
    git clone https://github.com/wow2006/lsd_slam.git
    chmod +x build.sh
    ./build.sh

# License
LSD-SLAM is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.

For commercial purposes, the original lsd slam authors also offer a professional version under different licencing terms.
