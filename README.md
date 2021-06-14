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

Download the dataset tools and download one or more ICL-NUIM datasets:

``` sh
cd /SOME/PATH
git clone git@bitbucket.org:smartroboticslab/dataset-tools.git
cd dataset-tools
./ICL-NUIM/icl-nuim-download.sh /ANOTHER/PATH
# You can stop the download with Ctrl+C after the second dataset has started
# downloading, only the first dataset will be used in this example.
```

Convert the downloaded dataset to the appropriate format:

``` sh
cd TUM/tum2raw
make
./bin/tum2raw /ANOTHER/PATH/living_room_traj0_frei_png
```

Copy the configuration file into the dataset folder and run supereight:

``` sh
cd /PATH/TO/supereight-2-srl
cp config/living_room_traj0_frei_png.yaml /ANOTHER/PATH/living_room_traj0_frei_png/config.yaml
./build/release/se_app/main_tsdf_multi /ANOTHER/PATH/living_room_traj0_frei_png/config.yaml
```

