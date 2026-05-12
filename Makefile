# WayWiseR Makefile
#
# Complete installation:
#   make all                     — configure + setup + build
#
# Step-by-step:
#   make configure               — (re-)configure .env interactively
#   make setup                   — install prereqs + venv + rosdep
#   make build                   — colcon build only
#   make rebuild                 — remove build/, install/, log/ then colcon build
#
# Cleanup:
#   make clean                   — remove build/, install/, log/ and .venv/
#   make clean ARGS=--skip-venv  — remove build/, install/, log/ (keep .venv)
#
# Options (via ARGS):
#   make all ARGS="--skip-ros"       — skip ROS2 installation
#   make all ARGS="--skip-mavsdk"    — skip MAVSDK installation
#   make all ARGS="--quiet"          — non-interactive
#
# Override workspace root:
#   WAYWISER_WS=/custom/path make all

# realpath resolves symlinks, so paths stay correct whether make is run from
# src/WayWiseR/ directly or from the workspace root via a Makefile symlink.
MAKEFILE_REAL  := $(realpath $(lastword $(MAKEFILE_LIST)))
WAYWISER_WS    ?= $(abspath $(dir $(MAKEFILE_REAL))/../..)
BOOTSTRAP      := $(dir $(MAKEFILE_REAL))bootstrap
ARGS           ?=

.PHONY: help all configure setup build rebuild test clean
.DEFAULT_GOAL := help

help:
	@echo "Workspace: $(WAYWISER_WS)"
	@echo ""
	@echo "Complete installation:"
	@echo "  make all             — configure + setup + build"
	@echo ""
	@echo "Step-by-step installation:"
	@echo "  make configure       — (re-)configure .env interactively"
	@echo "  make setup           — install prereqs + venv + rosdep"
	@echo "  make build           — colcon build"
	@echo "  make rebuild         — remove build/, install/, log/ then colcon build"
	@echo ""
	@echo "Testing:"
	@echo "  make test            — colcon test (WayWiseR packages only) + show results"
	@echo ""
	@echo "Cleanup:"
	@echo "  make clean           — remove build/, install/, log/ and .venv/"
	@echo ""
	@echo "Options (pass via ARGS):"
	@echo "  make all ARGS='--skip-ros'		— skip ROS2 installation"
	@echo "  make all ARGS='--skip-mavsdk'		— skip MAVSDK installation"
	@echo "  make all ARGS='--quiet'   		— non-interactive"
	@echo "  make clean ARGS='--skip-venv'  	— skip removing .venv/"

all:
	@WAYWISER_WS=$(WAYWISER_WS) bash $(BOOTSTRAP) $(ARGS)

configure:
	@WAYWISER_WS=$(WAYWISER_WS) bash $(BOOTSTRAP) --configure $(ARGS)

setup:
	@WAYWISER_WS=$(WAYWISER_WS) bash $(BOOTSTRAP) --setup-only $(ARGS)

build:
	@WAYWISER_WS=$(WAYWISER_WS) bash $(BOOTSTRAP) --build-only $(ARGS)

rebuild:
	@echo "Removing build/, install/, log/ ..."
	@rm -rf $(WAYWISER_WS)/build $(WAYWISER_WS)/install $(WAYWISER_WS)/log
	@WAYWISER_WS=$(WAYWISER_WS) bash $(BOOTSTRAP) --build-only $(ARGS)

test:
	@cd $(WAYWISER_WS) && \
	  . /opt/ros/humble/setup.sh && \
	  { . install/setup.bash 2>/dev/null || true; } && \
	  skipped="$${WAYWISER_SKIPPED_PACKAGES:-$$(grep '^WAYWISER_SKIPPED_PACKAGES=' .env 2>/dev/null | sed 's/^WAYWISER_SKIPPED_PACKAGES=//;s/"//g')}"; \
	  if [ -n "$$skipped" ]; then \
	    colcon test --base-paths src/WayWiseR/ --packages-skip $$skipped; \
	  else \
	    colcon test --base-paths src/WayWiseR/; \
	  fi; \
	  colcon test-result --verbose

clean:
	@echo "Removing build/, install/, log/ ..."
	@rm -rf $(WAYWISER_WS)/build $(WAYWISER_WS)/install $(WAYWISER_WS)/log
	@if echo " $(ARGS) " | grep -qv -- ' --skip-venv '; then \
		echo "Removing .venv/ ..."; \
		rm -rf $(WAYWISER_WS)/.venv; \
	fi
	@echo "Done. Run 'make all' to rebuild."
