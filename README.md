![Alt Text](docs/feature_matchings.gif)


# Installation 

## Standalone

### Dependencies

You need to install following libraries:

- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) - version 3.3.x
- [OpenCV](https://docs.opencv.org/3.4/d7/d9f/tutorial_linux_install.html) - version 3.4.1
- [Ceres](http://ceres-solver.org/installation.html) - version 1.14.0

You don't need to install following libraries since they are kept in thirdparth folder locally:

- [spdlog](https://github.com/gabime/spdlog) - version 1.x
- [moderncpp json](https://github.com/nlohmann/json) - version 3.4.0
- [args](https://github.com/Taywee/args) - version 6.0.4

### How to install

#### GCC, etc.
```
sudo apt install build-essential
```

#### OpenCV
```
sudo apt-get install build-essential checkinstall cmake pkg-config yasm
sudo apt-get install git gfortran
sudo apt-get install libjpeg8-dev libjasper-dev libpng12-dev
sudo apt-get install libtiff5-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
sudo apt-get install libxine2-dev libv4l-dev
sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install qt5-default libgtk2.0-dev libtbb-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
sudo apt-get install libvorbis-dev libxvidcore-dev
sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install x264 v4l-utils
git clone https://github.com/opencv/opencv.git
cd opencv 
git checkout 3.3.1 
cd ../
git clone https://github.com/opencv/opencv.git
cd opencv 
git checkout 3.3.1 
cd ../
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \ ..
make -j8
sudo checkinstall
```

#### Eigen3
```
sudo apt-get install libeigen3-dev
```

#### Ceres
```
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libsuitesparse-dev
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver/
mkdir build
cd build/
cmake ..
make -j8
sudo checkinstall
```

#### CoVO
```
git clone https://github.com/ugurbolat/CoVO.git
cd CoVO
mkdir build
cd build
cmake ..
make -j8
```
