# Supereight 2.0

## Install

**Clone the repo**
```shell
git clone git@bitbucket.org:smartroboticslab/supereight-2-srl.git
```

**Clone the submodules**
```shell
git submodule update --init --recursive
```

## Build

**Set the dataset, ground truth and output path in the `main_tsdf.cpp`**
```cpp
std::string output_path         = "PATH/TO/out";
reader_config.sequence_path     = "PATH/TO/scene.raw";
reader_config.ground_truth_file = "PATH/TO/scene.raw.txt";
```

**Compile code from root project folder**
```shell
make -j3 release
```

## Run

**Run example**
```shell
./build/release/se_app/main_tsdf
```
