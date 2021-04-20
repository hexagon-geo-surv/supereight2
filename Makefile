.PHONY: release
release: clean-cache
	mkdir -p build/release
	cd build/release && cmake -DCMAKE_BUILD_TYPE=Release $(CMAKE_ARGUMENTS) ../..
	$(MAKE) -C build/release $(MFLAGS)

.PHONY: relwithdebinfo
relwithdebinfo: clean-cache
	mkdir -p build/relwithdebinfo
	cd build/relwithdebinfo && cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo $(CMAKE_ARGUMENTS) ../..
	$(MAKE) -C build/relwithdebinfo $(MFLAGS)

.PHONY: debug
debug: clean-cache
	mkdir -p build/debug/logs
	cd build/debug && cmake -DCMAKE_BUILD_TYPE=Debug $(CMAKE_ARGUMENTS) ../..
	$(MAKE) -C build/debug $(MFLAGS)

.PHONY: install
install:
	$(MAKE) -C build/release $(MFLAGS) install

.PHONY: uninstall
uninstall:
	$(MAKE) -C build/release $(MFLAGS) uninstall



.PHONY: test
test: release
	$(MAKE) -C build/release $(MFLAGS) test

.PHONY: test-relwithdebinfo
test-relwithdebinfo: relwithdebinfo
	$(MAKE) -C build/relwithdebinfo $(MFLAGS) test

.PHONY: test-debug
test-debug: debug
	$(MAKE) -C build/debug $(MFLAGS) test

.PHONY: test-install
test-install:
	rm -rf build/testinstall
	mkdir -p build/testinstall
	cd build/testinstall && cmake -DCMAKE_BUILD_TYPE=Release $(CMAKE_ARGUMENTS) ../../test/test_install
	$(MAKE) -C build/testinstall $(MFLAGS)
	$(MAKE) -C build/testinstall $(MFLAGS) test



.PHONY: doc
doc:
	doxygen



.PHONY: clean
clean:
	rm -rf build

.PHONY: clean-cache
clean-cache:
	rm -rf build/release/CMakeCache.txt
	rm -rf build/relwithdebinfo/CMakeCache.txt
	rm -rf build/debug/CMakeCache.txt

.PHONY: clean-doc
clean-doc:
	rm -rf doc/html

