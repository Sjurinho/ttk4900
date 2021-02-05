MASTER THESIS: Tunnel based SLAM system for autonomous ground vehicles.

## Requirements:

### GTSAM: v4.0.0-alpha2
```
wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
cd ~/Downloads/gtsam-4.0.0-alpha2/
mkdir build && cd build
cmake ..
sudo make install
```
### PCL: v1.8
```
sudo apt install libpcl-dev
```

### Dynamic links I needed to make:
Symbolic links for packages like pcl and eigen, whose folders are shells to avoid nameconflicts. For example, when installing eigen3 through "sudo apt install libeigen3-dev"
```
cd /path/to/include/eigen3folder
sudo ln -sf eigen3/Eigen Eigen
```

The links i used was
```
eigen3/Eigen -> Eigen
pcl-1.8/pcl -> pcl
eigen3/unsupported -> unsupported
```

### GTSAM potential fix
If an error "cannot find -lBoost::timer" occurs, then this may fix it:
Go into /usr/local/lib/cmake/GTSAM/GTSAM-exports.cmake and change the linker line
```
INTERFACE_LINK_LIBRARIES "Boost::serialization;Boost::system;Boost::filesystem;Boost::thread;Boost::date_time;Boost::regex;Boost::timer;Boost::chrono;tbb;tbbmalloc;metis-gtsam
```
to 
```
INTERFACE_LINK_LIBRARIES "Boost::serialization;Boost::system;Boost::filesystem;Boost::thread;Boost::date_time;Boost::regex;/usr/lib/x86_64-linux-gnu/libboost_timer.so;Boost::chrono;tbb;tbbmalloc;metis-gtsam
```