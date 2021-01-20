MASTER THESIS: Tunnel based SLAM system for autonomous ground vehicles.

## Requirements:

### GTSAM: v4.0.2
```
wget -O ~/Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.2.zip
cd ~/Downloads/ && unzip gtsam.zip -d ~/Downloads/
cd ~/Downloads/gtsam-4.0.2/
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
sudo make install -j8
```
### PCL: v1.8
```
sudo apt install libpcl-dev
```

Notes to self:
Symbolic links for packages like pcl and eigen, whose folders are shells to avoid nameconflicts. For example, when installing eigen3 through "sudo install libeigen3-dev"
```
cd /path/to/include/eigen3folder
sudo ln -sf eigen3/Eigen Eigen
```