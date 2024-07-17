# SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-License-Identifier: CC0-1.0

.PHONY: release
release:
	mkdir -p build/release
	cd build/release && cmake -DCMAKE_BUILD_TYPE=Release ../..
	cmake --build build/release

.PHONY: devel
devel:
	mkdir -p build/devel
	cd build/devel && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ../..
	cmake --build build/devel

.PHONY: debug
debug:
	mkdir -p build/debug
	cd build/debug && cmake -DCMAKE_BUILD_TYPE=Debug ../..
	cmake --build build/debug



.PHONY: test
test: release
	cd build/release && ctest

.PHONY: test-devel
test-devel: devel
	cd build/devel && ctest

.PHONY: test-debug
test-debug: debug
	cd build/debug && ctest



.PHONY: install
install:
	cmake --install build/release

.PHONY: installcheck
installcheck:
	mkdir -p test/installcheck/build
	cd test/installcheck/build && cmake -DCMAKE_BUILD_TYPE=Release ..
	cmake --build test/installcheck/build
	./test/installcheck/build/installcheck



.PHONY: doc
doc:
	doxygen

.PHONY: format
format:
	find app include src test -type f -name '*.[ch]pp' ! -path '*/external/*' -exec clang-format-10 --style=file -i {} +

.PHONY: clean
clean:
	rm -rf build
	rm -rf test/installcheck/build

.PHONY: clean-cmake-cache
clean-cmake-cache:
	rm -rf build/debug/CMakeCache.txt
	rm -rf build/release/CMakeCache.txt
	rm -rf build/devel/CMakeCache.txt

.PHONY: clean-doc
clean-doc:
	rm -rf doc/html



.PHONY: download-icl-nuim
download-icl-nuim:
	mkdir -p datasets/ICL_NUIM
	./scripts/icl-nuim-download.sh datasets/ICL_NUIM 0
	@if [ -d datasets/ICL_NUIM/living_room_traj0_frei_png/ ]; then cp config/living_room_traj0_frei_png.yaml datasets/ICL_NUIM/living_room_traj0_frei_png/config.yaml; fi
	@if [ -d datasets/ICL_NUIM/living_room_traj1_frei_png/ ]; then cp config/living_room_traj1_frei_png.yaml datasets/ICL_NUIM/living_room_traj1_frei_png/config.yaml; fi
	@if [ -d datasets/ICL_NUIM/living_room_traj2_frei_png/ ]; then cp config/living_room_traj2_frei_png.yaml datasets/ICL_NUIM/living_room_traj2_frei_png/config.yaml; fi
	@if [ -d datasets/ICL_NUIM/living_room_traj3_frei_png/ ]; then cp config/living_room_traj3_frei_png.yaml datasets/ICL_NUIM/living_room_traj3_frei_png/config.yaml; fi
