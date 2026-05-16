# WayWiseR Makefile
#
# Complete installation:
#   make all                     — configure + setup + build + post-build
#
# Step-by-step:
#   make configure               — (re-)configure .env interactively
#   make setup                   — install prereqs + venv + rosdep
#   make build                   — colcon build only
#   make post-build              — run post-build setup hooks only
#   make rebuild                 — remove build/, install/, log*/ then build
#
# Cleanup:
#   make clean                   — remove build*/, install*/, log*/, deb_*/ and .venv/
#   make clean ARGS=--skip-venv  — remove build*/, install*/, log*/, deb_*/ (keep .venv)
#
# Options (via ARGS):
#   make all ARGS="--skip-mavsdk"      — skip MAVSDK installation
#   make all ARGS="--quiet"            — non-interactive
#   make package ARGS="amd64 arm64"    — build selected .deb package arches
#
# Override workspace root:
#   WAYWISER_WS=/custom/path make all

# realpath resolves symlinks, so paths stay correct whether make is run from
# src/WayWiseR/ directly or from the workspace root via a Makefile symlink.
MAKEFILE_REAL  := $(realpath $(lastword $(MAKEFILE_LIST)))
WAYWISER_WS    ?= $(abspath $(dir $(MAKEFILE_REAL))/../..)
BOOTSTRAP      := $(dir $(MAKEFILE_REAL))waywiser/scripts/bootstrap
CONFIGURE_ENV  := $(dir $(MAKEFILE_REAL))waywiser/scripts/configure_env.bash
PACKAGE_SCRIPT := $(dir $(MAKEFILE_REAL))waywiser/scripts/package
ARGS           ?=

# First-party packages — mirrors build.yaml package-name; used by 'make test'
# to avoid descending into submodules (PX4-Autopilot, ros_gz_harmonic, etc.).
WAYWISER_PACKAGES := \
  waywiser waywiser_agrarsense waywiser_carla waywiser_core \
  waywiser_description waywiser_gazebo waywiser_hwbringup waywiser_nav2 \
  waywiser_perception waywiser_rviz2 waywiser_slam waywiser_teleop \
  waywiser_test_runner waywiser_twist_safety

.PHONY: help all configure setup build post-build rebuild test package package-amd64 package-arm64 clean
.DEFAULT_GOAL := help

help:
	@echo "Workspace: $(WAYWISER_WS)"
	@echo ""
	@echo "Complete installation:"
	@echo "  make all             — configure + setup + build + post-build"
	@echo ""
	@echo "Step-by-step installation:"
	@echo "  make configure       — (re-)configure .env interactively"
	@echo "  make setup           — install prereqs + venv + rosdep"
	@echo "  make build           — colcon build only"
	@echo "  make post-build      — run post-build setup hooks only"
	@echo "  make rebuild         — remove build/, install/, log*/ then build"
	@echo ""
	@echo "Testing:"
	@echo "  make test            — colcon test (WayWiseR packages only) + show results"
	@echo ""
	@echo "Packages:"
	@echo "  make package         — build amd64 .deb packages in Docker"
	@echo "  make package ARGS='amd64 arm64' — build selected .deb package arches"
	@echo "  make package-amd64   — alias for: make package ARGS=amd64"
	@echo "  make package-arm64   — alias for: make package ARGS=arm64"
	@echo ""
	@echo "Cleanup:"
	@echo "  make clean           — remove build*/, install*/, log*/, deb_*/ and .venv/"
	@echo ""
	@echo "Options (pass via ARGS):"
	@echo "  make all ARGS='--skip-mavsdk'		— skip MAVSDK installation"
	@echo "  make all ARGS='--quiet'   		— non-interactive"
	@echo "  make package ARGS='amd64 arm64' 	— package selected architectures"
	@echo "  make clean ARGS='--skip-venv'  	— skip removing .venv/"

all:
	@WAYWISER_WS=$(WAYWISER_WS) bash $(BOOTSTRAP) $(ARGS)

configure:
	@WAYWISER_WS=$(WAYWISER_WS) \
	  WAYWISER_REPO_DIR=$(dir $(MAKEFILE_REAL)) \
	  WAYWISER_ENV_FILE=$(dir $(MAKEFILE_REAL)).env \
	  WAYWISER_ENV_FILE_EXAMPLE=$(dir $(MAKEFILE_REAL)).env.example \
	  bash $(CONFIGURE_ENV) $(ARGS)

setup:
	@WAYWISER_WS=$(WAYWISER_WS) bash $(BOOTSTRAP) --setup-only $(ARGS)

build:
	@WAYWISER_WS=$(WAYWISER_WS) bash $(BOOTSTRAP) --build-only $(ARGS)

post-build:
	@WAYWISER_WS=$(WAYWISER_WS) bash $(BOOTSTRAP) --post-build-only $(ARGS)

rebuild:
	@echo "Removing build/, install/, log/ ..."
	@rm -rf $(WAYWISER_WS)/build $(WAYWISER_WS)/install $(WAYWISER_WS)/log
	@WAYWISER_WS=$(WAYWISER_WS) bash $(BOOTSTRAP) --build-only $(ARGS)

test:
	$(eval SKIPPED := $(shell grep '^WAYWISER_SKIPPED_PACKAGES=' $(dir $(MAKEFILE_REAL)).env 2>/dev/null | sed 's/^WAYWISER_SKIPPED_PACKAGES=//;s/"//g'))
	$(eval SKIPPED := $(or $(WAYWISER_SKIPPED_PACKAGES),$(SKIPPED)))
	$(eval TEST_SELECTION := $(or $(WAYWISER_TEST_PACKAGES),$(WAYWISER_BUILD_PACKAGES),$(WAYWISER_PACKAGES)))
	$(eval PKG_LIST := $(filter-out $(SKIPPED),$(TEST_SELECTION)))
	@cd $(WAYWISER_WS) && bash -c '\
	  venv_site=$$(find .venv/lib -maxdepth 2 -type d -name site-packages 2>/dev/null | head -n 1); \
	  if [ -n "$$venv_site" ] && [ -n "$$PYTHONPATH" ]; then \
	    export PYTHONPATH=$$(printf "%s" "$$PYTHONPATH" | tr ":" "\n" | grep -vx "$$(pwd)/$$venv_site" | grep -vx "$$venv_site" | paste -sd: -); \
	  fi; \
	  . /opt/ros/jazzy/setup.bash; \
	  if [ -n "$$WAYWISER_UNDERLAY_SETUP" ] && [ -f "$$WAYWISER_UNDERLAY_SETUP" ]; then . "$$WAYWISER_UNDERLAY_SETUP"; fi; \
	  . install/setup.bash 2>/dev/null || true; \
	  colcon test-result --delete-yes >/dev/null 2>&1 || true; \
	  colcon test --packages-select $(PKG_LIST); \
	  colcon test-result --verbose'

package:
	@bash $(PACKAGE_SCRIPT) $(if $(strip $(ARGS)),$(ARGS),amd64)

package-amd64:
	@bash $(PACKAGE_SCRIPT) amd64

package-arm64:
	@bash $(PACKAGE_SCRIPT) arm64

clean:
	@echo "Removing build*/, install*/, log*/, deb_*/ ..."
	@rm -rf $(WAYWISER_WS)/build $(WAYWISER_WS)/build-* \
	  $(WAYWISER_WS)/install $(WAYWISER_WS)/install-* $(WAYWISER_WS)/log* \
	  $(WAYWISER_WS)/deb_dist $(WAYWISER_WS)/deb_install $(WAYWISER_WS)/deb_stage \
	  $(dir $(MAKEFILE_REAL))waywiser_core/external/PX4-Autopilot/build || { \
		echo "Permission denied while cleaning; retrying with sudo for files created by Docker/QEMU ..."; \
		sudo rm -rf $(WAYWISER_WS)/build $(WAYWISER_WS)/build-* \
		  $(WAYWISER_WS)/install $(WAYWISER_WS)/install-* $(WAYWISER_WS)/log* \
		  $(WAYWISER_WS)/deb_dist $(WAYWISER_WS)/deb_install $(WAYWISER_WS)/deb_stage \
		  $(dir $(MAKEFILE_REAL))waywiser_core/external/PX4-Autopilot/build; \
	  }
	@if echo " $(ARGS) " | grep -qv -- ' --skip-venv '; then \
		echo "Removing .venv/ ..."; \
		rm -rf $(WAYWISER_WS)/.venv || { \
			echo "Permission denied while cleaning .venv/; retrying with sudo ..."; \
			sudo rm -rf $(WAYWISER_WS)/.venv; \
		}; \
	fi
	@echo "Done. Run 'make all' to rebuild."
