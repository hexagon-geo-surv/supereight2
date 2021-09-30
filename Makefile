# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

.PHONY: release
release:
	mkdir -p build/release
	cd build/release && cmake -DCMAKE_BUILD_TYPE=Release ../..
	cmake --build build/release

.PHONY: relwithdebinfo
relwithdebinfo:
	mkdir -p build/relwithdebinfo
	cd build/relwithdebinfo && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ../..
	cmake --build build/relwithdebinfo

.PHONY: debug
debug:
	mkdir -p build/debug
	cd build/debug && cmake -DCMAKE_BUILD_TYPE=Debug ../..
	cmake --build build/debug



.PHONY: test
test: release
	cd build/release && ctest

.PHONY: test-relwithdebinfo
test-relwithdebinfo: relwithdebinfo
	cd build/relwithdebinfo && ctest

.PHONY: test-debug
test-debug: debug
	cd build/debug && ctest



.PHONY: doc
doc:
	doxygen

.PHONY: format
format:
	find include src test -regex '.*\.\(cpp\|hpp\|c\|h\)' -exec clang-format -style=file -i {} \;

.PHONY: clean
clean:
	rm -rf build

.PHONY: clean-cmake-cache
clean-cmake-cache:
	rm -rf build/debug/CMakeCache.txt
	rm -rf build/release/CMakeCache.txt
	rm -rf build/relwithdebinfo/CMakeCache.txt

.PHONY: clean-doc
clean-doc:
	rm -rf doc/html



.PHONY: download-icl-nuim
download-icl-nuim:
	mkdir -p datasets/ICL_NUIM
	./scripts/icl-nuim-download.sh datasets/ICL_NUIM
	cp config/living_room_traj0_frei_png.yaml datasets/ICL_NUIM/living_room_traj0_frei_png/config.yaml
	cp config/living_room_traj1_frei_png.yaml datasets/ICL_NUIM/living_room_traj1_frei_png/config.yaml
	cp config/living_room_traj2_frei_png.yaml datasets/ICL_NUIM/living_room_traj2_frei_png/config.yaml
	cp config/living_room_traj3_frei_png.yaml datasets/ICL_NUIM/living_room_traj3_frei_png/config.yaml
	mkdir -p datasets/ICL_NUIM/living_room_traj0_frei_png/out
	mkdir -p datasets/ICL_NUIM/living_room_traj1_frei_png/out
	mkdir -p datasets/ICL_NUIM/living_room_traj2_frei_png/out
	mkdir -p datasets/ICL_NUIM/living_room_traj3_frei_png/out

