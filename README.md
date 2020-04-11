# What?

**CoVO** is a Visual Odometry tool that uses RGB-D images to estimate the *relative pose* (position and orientation) of a camera and the *covariances* of pose estimation.

# Why?

**CoVO** was built because there was no Visual Odometry tool that can provide *uncertainty* of pose estimations.

# How?

**CoVO** uses sensor noise models of the camera to propagate the measurement errors (epistemic uncertainty) to estimate covariances.



![Alt Text](docs/feature_matchings.gif)


# Installation 

## Standalone

### Dependencies

You need to install following libraries:

- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) - version 3.3.x
- [OpenCV](https://docs.opencv.org/3.4/d7/d9f/tutorial_linux_install.html) - version 3.4.1
- [Ceres](http://ceres-solver.org/installation.html) - version 1.14.0

You don't need to install following libraries since they are kept in thirdparty folder locally:

- [spdlog](https://github.com/gabime/spdlog) - version 1.x
- [moderncpp json](https://github.com/nlohmann/json) - version 3.4.0
- [args](https://github.com/Taywee/args) - version 6.0.4

### How to install

#### OpenCV

Follow this tutorial to install [OpenCV](https://www.learnopencv.com/install-opencv3-on-ubuntu/)

**IMPORTANT**: Be careful with the version you install and pay attention as to which branch you are in before installing so don't forget to run the following command:

```
$ git checkout 3.3.1 
```

*RECOMMENDATION*: instead of using "make install" command, use 'checkinstall' which creates a '.deb' packages so that you easily uninstall it:

```
$ sudo checkinstall
```

#### Eigen3
```
$ sudo apt-get install libeigen3-dev
```

#### Ceres

Follow the official tutorial to install [Ceres-Solver](http://ceres-solver.org/installation.html)

**IMPORTANT**: Be careful with the version you install and pay attention as to which branch you are in before installing so don't forget to run the following command:

```
$ git checkout 1.14.0 
```

#### CoVO

Finally, to install CoVO:

```
git clone https://github.com/ugurbolat/CoVO.git
cd CoVO
mkdir build
cd build
cmake ..
make -j8
```
