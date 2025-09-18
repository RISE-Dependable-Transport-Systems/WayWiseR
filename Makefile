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

# ── Shell & make behavior ──────────────────────────────────────────────────────
SHELL := /bin/bash
.SHELLFLAGS := -eo pipefail -c
.ONESHELL:
MAKEFLAGS += --no-builtin-rules --no-print-directory

# Quiet by default; use `make V=1 …` for verbose command echo
ifeq ($(V),1)
    # verbose: let make echo recipes
else
    .SILENT:
endif

# ── Pretty printing helpers (robust quoting) ───────────────────────────────────
EMOJI_OK      := ✅
EMOJI_RUN     := ▶️
EMOJI_INFO    := ℹ️
EMOJI_WARN    := ⚠️
EMOJI_ROS     := 🤖
EMOJI_BOX     := 📦
EMOJI_GEAR    := 🛠️
EMOJI_CLEAN   := 🧹
EMOJI_LINK    := 🔗
EMOJI_CFG     := 🧩
EMOJI_DONE    := 🎉

BOLD := \033[1m
DIM  := \033[2m
CYAN := \033[36m
GREY := \033[90m
RESET:= \033[0m

define say
	printf '$(1)\n'
endef
define step
	printf '$(EMOJI_RUN)  $(BOLD)%s$(RESET)\n' '$(1)'
endef
define info
	printf '$(EMOJI_INFO)  %s\n' '$(1)'
endef
define warn
	printf '$(EMOJI_WARN)  %s\n' '$(1)'
endef
define ok
	printf '$(EMOJI_OK)  %s\n' '$(1)'
endef

# ── Config ─────────────────────────────────────────────────────────────────────
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
APTGET := $(SUDO) env DEBIAN_FRONTEND=noninteractive apt-get -y -q -o Dpkg::Use-Pty=0 -o Acquire::Retries=3 -o DPkg::Options::=--force-confnew -o DPkg::Options::=--force-confdef

# ── Pretty help (grouped to avoid reordering) ──────────────────────────────────
.PHONY: help
help: ## Show help and current settings
	@{ \
	  printf '$(BOLD)WayWiseR — Make targets$(RESET)\n\n'; \
	  printf '$(EMOJI_INFO)  Detected:\n'; \
	  printf '  container     : %s\n' '$(IS_CONTAINER)'; \
	  printf '  workspace     : %s\n' '$(WAYWISER_WS)'; \
	  printf '  ros setup     : %s (%s)\n' '$(ROS_SETUP)' "$$(test -f '$(ROS_SETUP)' && echo present || echo missing)"; \
	  printf '\n$(EMOJI_INFO)  Targets:\n'; \
	  awk -F':|##' '/^[a-zA-Z0-9_.-]+:.*##/{printf "  \033[36m%-18s\033[0m %s\n", $$1, $$3}' $(MAKEFILE_LIST) | sort; \
	  printf '\n$(EMOJI_INFO)  Variables (override with VAR=value):\n'; \
	  printf '  %-27s = %s\n' CHOOSE_ROS_DISTRO '$(CHOOSE_ROS_DISTRO)'; \
	  printf '  %-27s = %s\n' INSTALL_PACKAGE '$(INSTALL_PACKAGE)'; \
	  printf '  %-27s = %s\n' TARGET_OS '$(TARGET_OS)'; \
	  printf '  %-27s = %s\n' WAYWISER_WS '$(WAYWISER_WS)'; \
	  printf '  %-27s = %s\n' WAYWISER_SKIPPED_PACKAGES '$(WAYWISER_SKIPPED_PACKAGES)'; \
	  printf '  %-27s = %s\n' MAVSDK_VERSION '$(MAVSDK_VERSION)'; \
	  printf '\n$(EMOJI_INFO)  Tip: run with '\''V=1'\'' to see all commands.\n'; \
	}

# ── High-level flows ──────────────────────────────────────────────────────────
.PHONY: all auto host devcontainer
all: auto ## Auto-detect and run host/devcontainer flow

auto: ## Auto-detect: inside container with ROS? -> devcontainer, else host
	@if [ "$(IS_CONTAINER)" = yes ] && [ -f "$(ROS_SETUP)" ]; then \
	  $(call step,Auto-detect: container + ROS → devcontainer); \
	  $(MAKE) devcontainer; \
	else \
	  $(call step,Auto-detect: host or missing ROS → host); \
	  $(MAKE) host; \
	fi

# Host: add ROS repo before installing ROS-related tools
host: check-os deps-base maybe-ros repo deps-ros-tools maybe-udev-luxonis maybe-ros-install mavsdk rosdep-setup setup-core build env done ## Full host setup (installs ROS if missing)

# Devcontainer: ROS is present, but we still add repo so tools resolve on noble/arm
devcontainer: check-ros deps-base repo deps-ros-tools maybe-udev-luxonis mavsdk rosdep-setup setup-core build env done ## Devcontainer setup (skips ROS install, installs MAVSDK)

# ── Checks & ROS repo + deps split ────────────────────────────────────────────
.PHONY: check-os check-ros maybe-ros maybe-ros-install repo ros-install
.PHONY: deps-base deps-ros-tools

check-os: ## Verify Ubuntu 24.04 and 64-bit arch (host)
	$(call step,$(EMOJI_CFG) Checking OS & arch)
	if ! command -v lsb_release >/dev/null 2>&1; then $(APTGET) update && $(APTGET) install lsb-release curl; fi
	OS_CODENAME="$$(. /etc/os-release >/dev/null 2>&1; echo $$UBUNTU_CODENAME)"; [ -z "$$OS_CODENAME" ] && OS_CODENAME="$$(lsb_release -sc)"
	if [ "$$OS_CODENAME" != "$(TARGET_OS)" ]; then echo 'ERROR: Unsupported OS ('"$$OS_CODENAME"'). Expected $(TARGET_OS).' >&2; exit 1; fi
	ARCH="$$(dpkg --print-architecture 2>/dev/null || uname -m)"; echo "Arch: $$ARCH"
	if ! echo "$$ARCH" | grep -qE 'amd64|arm64'; then echo 'ERROR: need amd64/arm64' >&2; exit 1; fi
	$(call ok,OS check passed)

check-ros: ## Inform whether ROS is already present
	$(call step,$(EMOJI_ROS) Checking for ROS at $(ROS_SETUP))
	if [ -f "$(ROS_SETUP)" ]; then echo 'ROS found ✅'; else echo 'ROS not found (continuing)…'; fi

maybe-ros:
	$(call step,$(EMOJI_ROS) ROS presence check)
	if [ -f "$(ROS_SETUP)" ]; then echo '[ROS] Already present → skipping install.'; exit 0; fi

repo: ## Configure ROS 2 apt source (ros2-apt-source)
	$(call step,$(EMOJI_LINK) Adding ROS 2 apt source)
	$(APTGET) update
	$(APTGET) install software-properties-common curl ca-certificates gnupg
	$(SUDO) add-apt-repository -y universe
	# Install ros-apt-source meta pkg (tracks latest repo URL)
	ROS_APT_SOURCE_VERSION="$$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F\" '{print $$4}')"
	CODENAME="$$(. /etc/os-release && echo $$VERSION_CODENAME)"
	URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/$${ROS_APT_SOURCE_VERSION}/ros2-apt-source_$${ROS_APT_SOURCE_VERSION}.$${CODENAME}_all.deb"
	echo "Fetching $$URL"
	curl -fsSL -o /tmp/ros2-apt-source.deb "$$URL"
	$(SUDO) dpkg -i /tmp/ros2-apt-source.deb || true
	$(call ok,ROS apt source configured)

maybe-ros-install:
	$(call step,$(EMOJI_ROS) Maybe install ROS)
	if [ -f "$(ROS_SETUP)" ]; then echo "[ROS] Already present → skip 'ros-install'"; else $(MAKE) ros-install; fi

ros-install: ## Install ROS 2 base/desktop
	$(call step,$(EMOJI_ROS) Installing ROS 2 ($(CHOOSE_ROS_DISTRO):$(INSTALL_PACKAGE)))
	$(APTGET) update
	$(APTGET) upgrade
	$(APTGET) install ros-$(CHOOSE_ROS_DISTRO)-$(INSTALL_PACKAGE) ros-dev-tools
	if ! grep -F "source /opt/ros/$(CHOOSE_ROS_DISTRO)/setup.bash" /etc/bash.bashrc >/dev/null 2>&1; then \
	  echo "source /opt/ros/$(CHOOSE_ROS_DISTRO)/setup.bash" | $(SUDO) tee -a /etc/bash.bashrc >/dev/null; \
	fi
	$(call ok,ROS installed)

# Split deps: base first (Ubuntu), then ROS tooling (after repo)
deps-base: ## Base system tools (no ROS repo required)
	$(call step,$(EMOJI_BOX) Installing base system & Python deps)
	$(APTGET) update
	$(APTGET) install git build-essential cmake ninja-build pkg-config \
	  python3-pip python3-venv \
	  libunwind-dev libqt5serialport5-dev \
	  curl ca-certificates
	$(call ok,Base dependencies installed)

deps-ros-tools: ## ROS development tools (after ROS apt source)
	$(call step,$(EMOJI_BOX) Installing ROS dev tools (colcon/rosdep/vcstool))
	$(APTGET) update
	$(APTGET) install python3-colcon-common-extensions python3-rosdep python3-vcstool
	$(call ok,ROS dev tools installed)

# ── Optional Luxonis udev rule flag ────────────────────────────────
LUXONIS ?= 0

.PHONY: maybe-udev-luxonis
maybe-udev-luxonis:
ifeq ($(strip $(LUXONIS)),1)
	$(call info,Flag LUXONIS=1 → running udev-luxonis)
	$(MAKE) udev-luxonis
else
	$(call info,Skipping Luxonis udev rule — pass LUXONIS=1 to enable)
endif

# ── UDEV rules for Luxonis OAK-D (host only) ──────────────────────────────────
UDEV_RULE_FILE := /etc/udev/rules.d/80-movidius.rules
UDEV_RULE_LINE := SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"

.PHONY: udev-luxonis
udev-luxonis: ## Install udev rule for Luxonis OAK-D (03e7) on the host
	$(call step,$(EMOJI_CFG) Installing udev rule for Luxonis OAK-D (03e7))
	# Skip inside containers: udev is a host service
	if [ "$(IS_CONTAINER)" = yes ]; then
	  $(call info,Detected container → skipping udev rule \(host-only step\))
	  exit 0
	fi
	if ! command -v udevadm >/dev/null 2>&1; then
	  $(call warn,udevadm not found — is this a Linux host with udev? Skipping.)
	  exit 0
	fi
	set -e
	# Create/update rule only if needed
	if [ ! -f "$(UDEV_RULE_FILE)" ] || ! grep -F '$(UDEV_RULE_LINE)' "$(UDEV_RULE_FILE)" >/dev/null 2>&1; then
	  echo '$(UDEV_RULE_LINE)' | $(SUDO) tee "$(UDEV_RULE_FILE)" >/dev/null
	  $(SUDO) udevadm control --reload-rules
	  $(SUDO) udevadm trigger
	  $(call ok,Luxonis udev rule installed & udev reloaded)
	else
	  $(call ok,Luxonis udev rule already present — nothing to do)
	fi

# ── Common deps, rosdep, workspace, venv, build -------------------------------
.PHONY: rosdep-setup setup-core build env done

rosdep-setup: ## Initialize/perm/update rosdep
	$(call step,$(EMOJI_GEAR) Setting up rosdep)
	set -e
	if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then $(SUDO) rosdep init || true; fi
	$(SUDO) rosdep fix-permissions || true
	rosdep update
	$(call ok,rosdep ready)

setup-core: ## Workspace, submodules, venv, pip, rosdep install
	$(call step,$(EMOJI_LINK) Preparing workspace & submodules)
	if [ -f "$(ROS_SETUP)" ]; then source "$(ROS_SETUP)"; fi
	mkdir -p "$(WAYWISER_WS)/src"
	if [ ! -e "$(WAYWISER_WS)/src/WayWiseR" ]; then ln -sfn "$(PWD)" "$(WAYWISER_WS)/src/WayWiseR"; fi
	cd "$(WAYWISER_WS)/src/WayWiseR"
	git config -f .gitmodules submodule.waywiser_core/WayWise.url https://github.com/RISE-Dependable-Transport-Systems/WayWise.git || true
	git submodule sync --recursive
	git -c url.https://github.com/.insteadof=git@github.com: submodule update --init --recursive --jobs 4
	$(call ok,Workspace linked & submodules updated)

	$(call step,$(EMOJI_GEAR) Python venv & requirements)
	cd "$(WAYWISER_WS)"
	python3 -m venv .venv
	source .venv/bin/activate
	python -m pip install --upgrade pip -q
	python -m pip install -q -r src/WayWiseR/requirements.txt
	$(call ok,Python environment ready)

	$(call step,$(EMOJI_ROS) rosdep install for project)
	cd "$(WAYWISER_WS)"
	# Get package names, drop skipped ones, then map back to paths
	SEL_PKGS=$$(colcon list --names-only | { \
	  if [ -n "$(WAYWISER_SKIPPED_PACKAGES)" ]; then \
	    grep -Ev "^($$(echo $(WAYWISER_SKIPPED_PACKAGES) | tr ' ' '|'))$$"; \
	  else \
	    cat; \
	  fi; \
	})
	if [ -z "$$SEL_PKGS" ]; then \
	  echo "No packages selected after applying skip list — skipping rosdep."; \
	else \
	  PKG_PATHS=$$(colcon list --paths-only --packages-select $$SEL_PKGS); \
	  rosdep install --from-paths $$PKG_PATHS --ignore-src --rosdistro $(CHOOSE_ROS_DISTRO) -r -y || true; \
	fi
	$(call ok,rosdep install done)

build: ## colcon build (respects WAYWISER_SKIPPED_PACKAGES)
	$(call step,$(EMOJI_GEAR) Building with colcon)
	if [ -f "$(ROS_SETUP)" ]; then source "$(ROS_SETUP)"; fi
	cd "$(WAYWISER_WS)"
	colcon build --symlink-install --packages-skip $(WAYWISER_SKIPPED_PACKAGES)
	$(call ok,Build complete)

env: ## Persist environment to ~/.bashrc
	$(call step,$(EMOJI_CFG) Persisting environment to ~/.bashrc)
	if ! grep -F "export WAYWISER_WS=$(WAYWISER_WS)" $$HOME/.bashrc >/dev/null 2>&1; then echo "export WAYWISER_WS=$(WAYWISER_WS)" >> $$HOME/.bashrc; fi
	if ! grep -F "export WAYWISER_SKIPPED_PACKAGES=$(WAYWISER_SKIPPED_PACKAGES)" $$HOME/.bashrc >/dev/null 2>&1; then echo "export WAYWISER_SKIPPED_PACKAGES=\"$(WAYWISER_SKIPPED_PACKAGES)\"" >> $$HOME/.bashrc; fi
	if ! grep -F "source \$$WAYWISER_WS/install/local_setup.bash" $$HOME/.bashrc >/dev/null 2>&1; then echo "source \$$WAYWISER_WS/install/local_setup.bash" >> $$HOME/.bashrc; fi
	if ! grep -F "source \$$WAYWISER_WS/.venv/bin/activate" $$HOME/.bashrc >/dev/null 2>&1; then echo "source \$$WAYWISER_WS/.venv/bin/activate" >> $$HOME/.bashrc; fi
	$(call ok,Environment persisted)

.done-msg = printf '$(EMOJI_DONE) $(BOLD)%s$(RESET) $(DIM)%s$(RESET)\n' 'WayWiseR setup complete.' "Open a NEW shell or 'source ~/.bashrc'."

done: ## Finish message
	@$(.done-msg)

# ── MAVSDK install (host or container) ----------------------------------------
.PHONY: mavsdk
mavsdk: ## Install/upgrade libmavsdk-dev from GitHub release (idempotent)
	$(call step,$(EMOJI_BOX) Installing/Updating MAVSDK $(MAVSDK_VERSION))
	set -eu
	if [ "$$EUID" -ne 0 ] && ! command -v sudo >/dev/null 2>&1; then echo 'MAVSDK install requires root or sudo' >&2; exit 1; fi
	installed_ver="$$(dpkg-query -W -f='$${Version}\n' libmavsdk-dev 2>/dev/null || true)"
	if [ -n "$$installed_ver" ] && dpkg --compare-versions "$$installed_ver" ge "$(MAVSDK_VERSION)"; then echo "MAVSDK already installed ($$installed_ver) ≥ $(MAVSDK_VERSION) — skipping"; exit 0; fi
	base="https://github.com/mavlink/MAVSDK/releases/download/v$(MAVSDK_VERSION)"; arch="$$(dpkg --print-architecture)"
	if [ -n "$(MAVSDK_PKG_OVERRIDE)" ]; then pkg="$(MAVSDK_PKG_OVERRIDE)"; \
	  else case "$$arch" in \
	    amd64) pkg="libmavsdk-dev_$(MAVSDK_VERSION)_ubuntu22.04_amd64.deb" ;; \
	    arm64) pkg="libmavsdk-dev_$(MAVSDK_VERSION)_debian12_arm64.deb" ;; \
	    *) echo "Unsupported arch: $$arch" >&2; exit 1 ;; \
	  esac; fi
	echo "Installing MAVSDK $(MAVSDK_VERSION) ($$pkg)"
	curl -fsSL -o /tmp/mavsdk.deb "$$base/$$pkg"
	$(APTGET) update
	$(SUDO) dpkg -i /tmp/mavsdk.deb || { $(APTGET) -f install; $(SUDO) dpkg -i /tmp/mavsdk.deb; }
	rm -f /tmp/mavsdk.deb
	$(call ok,MAVSDK installed)

# ── Maintenance ───────────────────────────────────────────────────────────────
.PHONY: clean uninstall post-attach
clean: ## Remove build/install/logs in workspace
	$(call step,$(EMOJI_CLEAN) Cleaning workspace artifacts)
	cd "$(WAYWISER_WS)" 2>/dev/null || exit 0
	rm -rf "$(WAYWISER_WS)/build" "$(WAYWISER_WS)/install" "$(WAYWISER_WS)/log"
	$(call ok,Workspace clean)

uninstall: ## Remove ROS (host) + autoremove (dangerous, optional)
	$(call warn,Removing ROS $(CHOOSE_ROS_DISTRO) (dangerous))
	$(SUDO) apt-get remove -y "~nros-$(CHOOSE_ROS_DISTRO)-*" || true
	$(APTGET) autoremove
	$(SUDO) apt-get remove -y ros2-apt-source || true
	$(APTGET) update && $(APTGET) upgrade
	$(call ok,ROS $(CHOOSE_ROS_DISTRO) removed)

post-attach: ## Mirror prior post-attach echo
	$(call say,WayWiseR environment ready. Tip: open a NEW terminal so the overlay is sourced.)
