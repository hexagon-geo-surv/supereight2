# Supereight 2.0



## Build

Install the dependencies

* GCC 7+ or clang 6+
* CMake 3.10+
* Eigen 3
* OpenCV 3+
* GLut (optional, for the GUI)
* OpenNI2 (optional, for Microsoft Kinect/Asus Xtion input)
* Make (optional, for convenience)

On Debian/Ubuntu you can install all of the above by running:

``` sh
sudo apt --yes install git g++ cmake libeigen3-dev libopencv-dev freeglut3-dev libopenni2-dev make
```

Clone the repository and its submodules:

``` sh
git clone --recurse-submodules git@bitbucket.org:smartroboticslab/supereight-2-srl.git
cd supereight-2-srl
# If you cloned without the --recurse-submodules run the following command:
git submodule update --init --recursive
```

Build in release mode:

``` sh
make
# Or if you don't have/like Make do a standard CMake build
mkdir -p build/release
cd build/release
cmake -DCMAKE_BUILD_TYPE=Release ../..
cmake --build .
```



## Usage example

Download the ICL-NUIM datasets:

``` sh
make download-icl-nuim 
```

Copy the configuration file into the dataset folder and run supereight:

``` sh
./build/release/se_app/main_tsdf_multi PATH/TO/dataset/living_room_traj0_frei_png/config.yaml
```

