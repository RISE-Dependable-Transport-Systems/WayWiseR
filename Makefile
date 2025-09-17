# WayWiseR — unified Makefile for devcontainer & host
# ---------------------------------------------------
# If ROS 2 already exists (e.g., osrf/ros:jazzy-* base image), we skip ROS install.
# 'make devcontainer' also installs/updates MAVSDK (moved out of Dockerfile).
#
# Usage:
#   make                # auto-detect container vs host
#   make devcontainer   # assumes ROS already present in image
#   make host           # installs ROS if missing
#   make mavsdk         # (re)install MAVSDK
#   make clean          # wipe build/install/log

SHELL := /bin/bash
.SHELLFLAGS := -eo pipefail -c
.ONESHELL:

# --- Config ---------------------------------------------------------------
CHOOSE_ROS_DISTRO ?= jazzy
INSTALL_PACKAGE   ?= desktop
TARGET_OS         ?= noble
MAVSDK_VERSION    ?= 2.14.1
MAVSDK_PKG_OVERRIDE ?=

# Detect container vs host and set sensible workspace default
IS_CONTAINER := $(shell { [ -f /.dockerenv ] || [ -f /run/.containerenv ] || [ -n "$$DEVCONTAINER" ]; } && echo yes || echo no)
WAYWISER_WS ?= $(shell if [ -n "$$WAYWISER_WS" ]; then echo "$$WAYWISER_WS"; elif [ "$(IS_CONTAINER)" = yes ]; then echo /workspaces/waywiser_ws; else echo "$$HOME/waywiser_ws"; fi)
WAYWISER_SKIPPED_PACKAGES ?= waywiser_agrarsense waywiser_carla waywiser_gazebo
ROS_SETUP ?= /opt/ros/$(CHOOSE_ROS_DISTRO)/setup.bash

# sudo only when needed
SUDO := $(shell if [ "$$EUID" -ne 0 ] && command -v sudo >/dev/null 2>&1; then echo sudo; fi)
APTGET := $(SUDO) env DEBIAN_FRONTEND=noninteractive apt-get -y -o Dpkg::Use-Pty=0 -o Acquire::Retries=3 -o DPkg::Options::=--force-confnew -o DPkg::Options::=--force-confdef

# Pretty help
.PHONY: help
help: ## Show help and current settings
	@echo "WayWiseR — Make targets"; echo
	@echo "Detected:"
	@printf "  container     : %s\n" "$(IS_CONTAINER)"
	@printf "  workspace     : %s\n" "$(WAYWISER_WS)"
	@printf "  ros setup     : %s (%s)\n" "$(ROS_SETUP)" "$$(test -f '$(ROS_SETUP)' && echo present || echo missing)"
	@echo
	@echo "Targets:"
	@awk -F':|##' '/^[a-zA-Z0-9_.-]+:.*##/{printf "  \033[36m%-18s\033[0m %s\n", $$1, $$3}' $(MAKEFILE_LIST) | sort
	@echo
	@echo "Variables (override with VAR=value):"
	@printf "  %-27s = %s\n" CHOOSE_ROS_DISTRO $(CHOOSE_ROS_DISTRO)
	@printf "  %-27s = %s\n" INSTALL_PACKAGE $(INSTALL_PACKAGE)
	@printf "  %-27s = %s\n" TARGET_OS $(TARGET_OS)
	@printf "  %-27s = %s\n" WAYWISER_WS $(WAYWISER_WS)
	@printf "  %-27s = %s\n" WAYWISER_SKIPPED_PACKAGES "$(WAYWISER_SKIPPED_PACKAGES)"
	@printf "  %-27s = %s\n" MAVSDK_VERSION $(MAVSDK_VERSION)

# --- High-level flows -----------------------------------------------------
.PHONY: all auto host devcontainer
all: auto ## Auto-detect and run host/devcontainer flow

auto: ## Auto-detect: inside container with ROS? -> devcontainer, else host
	@if [ "$(IS_CONTAINER)" = yes ] && [ -f "$(ROS_SETUP)" ]; then \
	  echo "[auto] Detected container with ROS -> make devcontainer"; \
	  $(MAKE) devcontainer; \
	else \
	  echo "[auto] Host or missing ROS -> make host"; \
	  $(MAKE) host; \
	fi

host: check-os deps maybe-ros repo maybe-ros-install mavsdk rosdep-setup setup-core build env done ## Full host setup (installs ROS if missing)

devcontainer: check-ros deps mavsdk rosdep-setup setup-core build env done ## Devcontainer setup (skips ROS install, installs MAVSDK)

# --- Checks ---------------------------------------------------------------
.PHONY: check-os check-ros
check-os: ## Verify Ubuntu 24.04 and 64-bit arch (host)
	if ! command -v lsb_release >/dev/null 2>&1; then $(APTGET) update && $(APTGET) install lsb-release curl; fi
	OS_CODENAME="$$(. /etc/os-release >/dev/null 2>&1; echo $$UBUNTU_CODENAME)"; [ -z "$$OS_CODENAME" ] && OS_CODENAME="$$(lsb_release -sc)"
	if [ "$$OS_CODENAME" != "$(TARGET_OS)" ]; then \
	  echo "ERROR: Unsupported OS ($$OS_CODENAME). Expected $(TARGET_OS)."; exit 1; \
	fi
	ARCH="$$(dpkg --print-architecture 2>/dev/null || uname -m)"; echo "Arch: $$ARCH"
	if ! echo "$$ARCH" | grep -qE 'amd64|arm64'; then echo "ERROR: need amd64/arm64"; exit 1; fi

check-ros: ## Inform whether ROS is already present
	if [ -f "$(ROS_SETUP)" ]; then echo "ROS found at $(ROS_SETUP)"; else echo "ROS not found (will continue)"; fi

# --- ROS install path (host only, and only if missing) -------------------
.PHONY: maybe-ros maybe-ros-install repo ros-install
maybe-ros:
	@if [ -f "$(ROS_SETUP)" ]; then \
	  echo "[ROS] Already present -> skipping install."; \
	  exit 0; \
	fi

repo: ## Configure ROS 2 apt source (ros2-apt-source)
	$(APTGET) update
	$(APTGET) install software-properties-common curl ca-certificates
	$(SUDO) add-apt-repository -y universe
	ROS_APT_SOURCE_VERSION="$$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F\" '{print $$4}')"
	CODENAME="$$(. /etc/os-release && echo $$VERSION_CODENAME)"
	URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/$${ROS_APT_SOURCE_VERSION}/ros2-apt-source_$${ROS_APT_SOURCE_VERSION}.$${CODENAME}_all.deb"
	echo "Fetching $${URL}"
	curl -fsSL -o /tmp/ros2-apt-source.deb "$${URL}"
	$(SUDO) dpkg -i /tmp/ros2-apt-source.deb || true

maybe-ros-install:
	@if [ -f "$(ROS_SETUP)" ]; then \
	  echo "[ROS] Already present -> skip 'ros-install'"; \
	else \
	  $(MAKE) ros-install; \
	fi

ros-install: ## Install ROS 2 base/desktop
	$(APTGET) update
	$(APTGET) upgrade
	$(APTGET) install ros-$(CHOOSE_ROS_DISTRO)-$(INSTALL_PACKAGE) ros-dev-tools
	# source for root shells
	if ! grep -F "source /opt/ros/$(CHOOSE_ROS_DISTRO)/setup.bash" /etc/bash.bashrc >/dev/null 2>&1; then \
	  echo "source /opt/ros/$(CHOOSE_ROS_DISTRO)/setup.bash" | $(SUDO) tee -a /etc/bash.bashrc >/dev/null; \
	fi

# --- Common deps, rosdep, workspace, venv, build -------------------------
.PHONY: deps rosdep-setup setup-core build env done

deps: ## System tools and Python tooling
	$(APTGET) update
	$(APTGET) install git build-essential cmake ninja-build pkg-config \
	  python3-pip python3-venv python3-colcon-common-extensions \
	  python3-rosdep python3-vcstool \
	  libunwind-dev libqt5serialport5-dev \
	  curl ca-certificates

rosdep-setup: ## Initialize/perm/update rosdep
	set -e
	if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then $(SUDO) rosdep init || true; fi
	$(SUDO) rosdep fix-permissions || true
	rosdep update

setup-core: ## Workspace, submodules, venv, pip, rosdep install
	# Source ROS env if available
	if [ -f "$(ROS_SETUP)" ]; then source "$(ROS_SETUP)"; fi
	# Workspace & link
	mkdir -p "$(WAYWISER_WS)/src"
	if [ ! -e "$(WAYWISER_WS)/src/WayWiseR" ]; then ln -sfn "$(PWD)" "$(WAYWISER_WS)/src/WayWiseR"; fi
	# Git HTTPS to avoid SSH in submodules
	git config --global url."https://github.com/".insteadof git@github.com:
	# Ensure WayWise submodule is HTTPS and sync/update
	cd "$(WAYWISER_WS)/src/WayWiseR"
	git config -f .gitmodules submodule.waywiser_core/WayWise.url https://github.com/RISE-Dependable-Transport-Systems/WayWise.git || true
	git submodule sync --recursive
	git submodule update --init --recursive --jobs 4
	# Python venv + pip reqs
	cd "$(WAYWISER_WS)"
	python3 -m venv .venv
	source .venv/bin/activate
	python -m pip install --upgrade pip
	python -m pip install -r src/WayWiseR/requirements.txt
	# rosdep install for packages (respect skip list)
	cd "$(WAYWISER_WS)"
	PKG_PATHS=$$(colcon list --paths-only | { [ -n "$(WAYWISER_SKIPPED_PACKAGES)" ] && grep -Ev "^($$(echo $(WAYWISER_SKIPPED_PACKAGES) | tr ' ' '|'))$$" || cat; })
	rosdep install --from-paths $$PKG_PATHS --ignore-src --rosdistro $(CHOOSE_ROS_DISTRO) -r -y || true

build: ## colcon build (respects WAYWISER_SKIPPED_PACKAGES)
	# Source ROS env & build
	if [ -f "$(ROS_SETUP)" ]; then source "$(ROS_SETUP)"; fi
	cd "$(WAYWISER_WS)"
	colcon build --symlink-install --packages-skip $(WAYWISER_SKIPPED_PACKAGES)

env: ## Persist environment to ~/.bashrc
	if ! grep -F "export WAYWISER_WS=$(WAYWISER_WS)" $$HOME/.bashrc >/dev/null 2>&1; then echo "export WAYWISER_WS=$(WAYWISER_WS)" >> $$HOME/.bashrc; fi
	if ! grep -F "export WAYWISER_SKIPPED_PACKAGES=$(WAYWISER_SKIPPED_PACKAGES)" $$HOME/.bashrc >/dev/null 2>&1; then echo "export WAYWISER_SKIPPED_PACKAGES=\"$(WAYWISER_SKIPPED_PACKAGES)\"" >> $$HOME/.bashrc; fi
	if ! grep -F "source \$$WAYWISER_WS/install/local_setup.bash" $$HOME/.bashrc >/dev/null 2>&1; then echo "source \$$WAYWISER_WS/install/local_setup.bash" >> $$HOME/.bashrc; fi
	if ! grep -F "source \$$WAYWISER_WS/.venv/bin/activate" $$HOME/.bashrc >/dev/null 2>&1; then echo "source \$$WAYWISER_WS/.venv/bin/activate" >> $$HOME/.bashrc; fi

.done-msg = echo "✅ WayWiseR setup complete. Open a NEW shell to load the overlay (or 'source ~/.bashrc')."

done: ## Finish message
	@$(.done-msg)

# --- MAVSDK install (host or container) ----------------------------------
.PHONY: mavsdk
mavsdk: ## Install/upgrade libmavsdk-dev from GitHub release (idempotent)
	set -eu
	# Require root or sudo
	if [ "$$EUID" -ne 0 ] && ! command -v sudo >/dev/null 2>&1; then \
	  echo "MAVSDK install requires root or sudo"; exit 1; \
	fi
	# Skip if already at desired version or newer
	installed_ver="$$(dpkg-query -W -f='$${Version}\n' libmavsdk-dev 2>/dev/null || true)"
	if [ -n "$$installed_ver" ] && dpkg --compare-versions "$$installed_ver" ge "$(MAVSDK_VERSION)"; then \
	  echo "MAVSDK already installed ($$installed_ver) ≥ $(MAVSDK_VERSION) — skipping"; \
	  exit 0; \
	fi
	base="https://github.com/mavlink/MAVSDK/releases/download/v$(MAVSDK_VERSION)"
	arch="$$(dpkg --print-architecture)"
	if [ -n "$(MAVSDK_PKG_OVERRIDE)" ]; then pkg="$(MAVSDK_PKG_OVERRIDE)"; \
	else case "$$arch" in \
	  amd64) pkg="libmavsdk-dev_$(MAVSDK_VERSION)_ubuntu22.04_amd64.deb" ;; \
	  arm64) pkg="libmavsdk-dev_$(MAVSDK_VERSION)_debian12_arm64.deb" ;; \
	  *) echo "Unsupported arch: $$arch"; exit 1 ;; \
	esac; fi
	echo "Installing MAVSDK $(MAVSDK_VERSION) ($$pkg)"
	curl -fsSL -o /tmp/mavsdk.deb "$$base/$$pkg"
	$(APTGET) update
	$(SUDO) dpkg -i /tmp/mavsdk.deb || { $(APTGET) -f install; $(SUDO) dpkg -i /tmp/mavsdk.deb; }
	rm -f /tmp/mavsdk.deb

# --- Maintenance ----------------------------------------------------------
.PHONY: clean uninstall post-attach
clean: ## Remove build/install/logs in workspace
	cd "$(WAYWISER_WS)" 2>/dev/null || exit 0
	rm -rf "$(WAYWISER_WS)/build" "$(WAYWISER_WS)/install" "$(WAYWISER_WS)/log"

uninstall: ## Remove ROS (host) + autoremove (dangerous, optional)
	$(SUDO) apt-get remove -y "~nros-$(CHOOSE_ROS_DISTRO)-*" || true
	$(APTGET) autoremove
	$(SUDO) apt-get remove -y ros2-apt-source || true
	$(APTGET) update && $(APTGET) upgrade
	echo "ROS $(CHOOSE_ROS_DISTRO) removed."

post-attach: ## Mirror prior post-attach echo
	@echo "WayWiseR environment ready. Tip: open a NEW terminal so the overlay is sourced."
