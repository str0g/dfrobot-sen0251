.DEFAULT_GOAL := help

BUILD_DIR := build

.PHONY: all list_presets test help pkg

all: debug

help:
	@echo "Usage:"
	@echo "  make debug     		- Configure & build using Debug preset"
	@echo "  make release   		- Configure & build using Release preset"
	@echo "  make clean     		- Remove build directories"
	@echo "  make pkg           - Builds pacman package for ArchLinux"
	@echo "  make help      		- Show this help message"

list_presets:
	cmake --list-presets

%:
	cmake --preset $@
	cmake --build ${BUILD_DIR}/$@
	ln -sf ${BUILD_DIR}/$@/compile_commands.json .

test:
	cmake --build build/debug/ && ctest --test-dir build/debug/ --output-on-failure

clean:
	rm -rf ${BUILD_DIR}

pkg:
	ln -sf package/PKGBUILD .
	makepkg -e -f
