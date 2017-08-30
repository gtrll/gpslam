GP-SLAM
===================================================
GP-SLAM is a library implenmenting sparse Gaussian process (GP) regression for continuous-time trajectory estimation and mapping. The core library is developed by C++ language, and an optional Matlab toolbox is also provided. Examples are provided in Matlab scripts.

GP-SLAM is being developed by [Jing Dong](mailto:thu.dongjing@gmail.com) and [Xinyan Yan](mailto:voidpointeryan@gmail.com) as part of their work at Georgia Tech Robot Learning Lab. 

Prerequisites
------

- CMake >= 2.6 (Ubuntu: `sudo apt-get install cmake`), compilation configuration tool.
- [Boost](http://www.boost.org/) >= 1.46 (Ubuntu: `sudo apt-get install libboost-all-dev`), portable C++ source libraries.
- [GTSAM](https://bitbucket.org/gtborg/gtsam) >= 4.0 alpha, a C++ library that implement smoothing and mapping (SAM) in robotics and vision.

Compilation & Installation
------

In the library folder excute:

```
$ mkdir build
$ cd build
$ cmake ..
$ make check  # optonal, run unit tests
$ make install
```

Matlab Toolbox
-----

An optional Matlab toolbox is provided to use our library in Matlab. To enable Matlab toolbox during compilation:

```
$ cmake -DGPSLAM_BUILD_MATLAB_TOOLBOX:OPTION=ON -DGTSAM_TOOLBOX_INSTALL_PATH:PATH=/path/install/toolbox ..
$ make install
```

After you install the Matlab toolbox, don't forget to add your `/path/install/toolbox` to your Matlab path.

Compatibility
-----

The GP-SLAM library is designed to be cross-platform, but it has been only tested on Ubuntu Linux for now.

Tested Compilers: 

- GCC 4.8, 5.4

Tested Boost version: 1.48-1.61

Linking to External Projects
-----

We provide easy linking to external **CMake** projects. Add following lines to your CMakeLists.txt

```
find_package(gpslam REQUIRED)
include_directories(${gpslam_INCLUDE_DIR})
```

Questions & Bug reporting
-----

Please use Github issue tracker to report bugs. For other questions please contact [Jing Dong](mailto:thu.dongjing@gmail.com).


Citing
-----

If you use GP-SLAM in an academic context, please cite following publications:

```
@inproceedings{Yan17ras,  
  Author = "Xinyan Yan and Vadim Indelman and Byron Boots",
  journal = " Robotics and Autonomous Systems",
  Title = "Incremental Sparse {GP} Regression for Continuous-time Trajectory Estimation and Mapping",
  Year = {2017},
  pages="120-132",
  volume = {87}
}
@article{Dong17arxiv,
  author    = {Jing Dong and Byron Boots and Frank Dellaert},
  title     = {Sparse Gaussian Processes for Continuous-Time Trajectory Estimation on Matrix Lie Groups},
  journal   = {Arxiv},
  volume    = {abs/1705.06020},
  year      = {2017},
  url       = {http://arxiv.org/abs/1705.06020}
}
```


License
-----

GP-SLAM is released under the BSD license, reproduced in the file LICENSE in this directory.
