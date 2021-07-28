.PHONY: release
release:
	cmake -DCMAKE_BUILD_TYPE=Release -S . -B build/release
	cmake --build build/release

.PHONY: relwithdebinfo
relwithdebinfo:
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -S . -B build/relwithdebinfo
	cmake --build build/relwithdebinfo

.PHONY: debug
debug:
	cmake -DCMAKE_BUILD_TYPE=Debug -S . -B build/debug
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

