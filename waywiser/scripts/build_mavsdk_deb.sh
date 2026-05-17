#!/usr/bin/env bash
set -euo pipefail

# Redirect all diagnostic output to stderr so that only the final deb path
# is written to stdout.  This is important when the script is called via
# command substitution (e.g. stage_mavsdk_deb in the package script).
exec 3>&1 1>&2

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
waywiser_ws="${WAYWISER_WS:-$(cd "$script_dir/../../../.." && pwd)}"
arch="${1:-$(dpkg --print-architecture 2>/dev/null || uname -m)}"
mavsdk_url="${MAVSDK_GIT_URL:-https://github.com/mavlink/MAVSDK.git}"
mavsdk_dir="${MAVSDK_SOURCE_DIR:-$waywiser_ws/build/MAVSDK}"
out_dir="${MAVSDK_DEB_OUT:-$waywiser_ws/deb_stage/$arch}"
ref="${MAVSDK_REF:-}"
mavsdk_release_version="${MAVSDK_VERSION_DEFAULT:-2.14.1}"
mavsdk_default_ref="${MAVSDK_DEFAULT_REF:-v${mavsdk_release_version}}"
cmake_version="3.25.3"

case "$arch" in
    amd64|x86_64) arch="amd64" ;;
    arm64|aarch64) arch="arm64" ;;
    armv7|armhf|armv7l) arch="armv7" ;;
    *) echo "Usage: $0 [amd64|arm64|armv7]" >&2; exit 2 ;;
esac

info() { echo "  $*"; }

resolve_mavsdk_project_dir() {
    if [[ -f "$mavsdk_dir/cpp/CMakeLists.txt" ]]; then
        printf '%s\n' "$mavsdk_dir/cpp"
        return 0
    fi

    if [[ -f "$mavsdk_dir/CMakeLists.txt" ]]; then
        printf '%s\n' "$mavsdk_dir"
        return 0
    fi

    return 1
}

resolve_mavsdk_package_tool() {
    local project_dir="$1"

    if [[ -x "$project_dir/tools/create_packages.sh" ]]; then
        printf '%s\n' "tools/create_packages.sh"
        return 0
    fi

    if [[ -x "$mavsdk_dir/tools/create_packages.sh" ]]; then
        printf '%s\n' "$(realpath --relative-to="$project_dir" "$mavsdk_dir/tools/create_packages.sh")"
        return 0
    fi

    return 1
}

cmake_linux_installer_arch() {
    case "$1" in
        amd64) printf '%s\n' "x86_64" ;;
        arm64) printf '%s\n' "aarch64" ;;
        *) return 1 ;;
    esac
}

cleanup_mavsdk_git_state() {
    local git_dir="$mavsdk_dir/.git"
    [[ -d "$git_dir" ]] || return 0

    find "$git_dir" -type f \( -name 'FETCH_HEAD' -o -name '*.lock' \) -delete 2>/dev/null || true
}

detect_waywise_ref() {
    local waywise_dir
    waywise_dir="$(cd "$script_dir/../../waywiser_core/WayWise" && pwd)"

    local branch
    branch="$(git -C "$waywise_dir" branch --show-current 2>/dev/null || true)"
    if [[ -n "$branch" ]]; then
        printf '%s\n' "$branch"
        return 0
    fi

    local tag
    tag="$(git -C "$waywise_dir" describe --tags --exact-match 2>/dev/null || true)"
    if [[ -n "$tag" ]]; then
        printf '%s\n' "$tag"
        return 0
    fi

    local superproject
    superproject="$(git -C "$waywise_dir" rev-parse --show-superproject-working-tree 2>/dev/null || true)"
    if [[ -n "$superproject" ]]; then
        branch="$(git -C "$superproject" branch --show-current 2>/dev/null || true)"
        if [[ -n "$branch" ]]; then
            printf '%s\n' "$branch"
        fi
    fi
}

ensure_mavsdk_checkout() {
    if [[ -d "$mavsdk_dir/.git" ]] && ! git -C "$mavsdk_dir" rev-parse --git-dir >/dev/null 2>&1; then
        info "MAVSDK checkout is corrupted, removing and re-cloning..."
        rm -rf "$mavsdk_dir"
    fi

    if [[ ! -d "$mavsdk_dir/.git" ]]; then
        info "Cloning MAVSDK into $mavsdk_dir..."
        git clone --recursive "$mavsdk_url" "$mavsdk_dir"
    else
        info "Updating MAVSDK checkout in $mavsdk_dir..."
        cleanup_mavsdk_git_state
        git -C "$mavsdk_dir" fetch --tags origin
        git -C "$mavsdk_dir" submodule sync --recursive
    fi

    if [[ -z "$ref" ]]; then
        ref="$(detect_waywise_ref || true)"
    fi

    if [[ -n "$ref" ]] && git -C "$mavsdk_dir" rev-parse --verify --quiet "origin/$ref" >/dev/null; then
        info "Using MAVSDK branch origin/$ref."
        git -C "$mavsdk_dir" checkout -B "$ref" "origin/$ref"
    elif [[ -n "$ref" ]] && git -C "$mavsdk_dir" rev-parse --verify --quiet "$ref" >/dev/null; then
        info "Using MAVSDK ref $ref."
        git -C "$mavsdk_dir" checkout "$ref"
    elif [[ -n "$mavsdk_default_ref" ]] \
        && git -C "$mavsdk_dir" rev-parse --verify --quiet "$mavsdk_default_ref" >/dev/null; then
        info "No matching MAVSDK ref found for '${ref:-<none>}'; using default MAVSDK ref $mavsdk_default_ref."
        git -C "$mavsdk_dir" checkout "$mavsdk_default_ref"
        ref="$mavsdk_default_ref"
    else
        info "No matching MAVSDK ref found for '${ref:-<none>}'; using existing/default MAVSDK checkout."
    fi

    git -C "$mavsdk_dir" submodule update --init --recursive
}

build_amd64() {
    local project_dir package_tool
    project_dir="$(resolve_mavsdk_project_dir)" || {
        echo "ERROR: No MAVSDK CMakeLists.txt found in $mavsdk_dir or $mavsdk_dir/cpp" >&2
        exit 1
    }
    package_tool="$(resolve_mavsdk_package_tool "$project_dir")" || {
        echo "ERROR: MAVSDK package helper not found under $mavsdk_dir" >&2
        exit 1
    }

    cd "$project_dir"
    rm -rf build/release
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_MAVSDK_SERVER=OFF -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX=install -DWERROR=OFF -S . -Bbuild/release
    cmake --build build/release --target install -- -j5
    "$package_tool" ./install . amd64 libmavsdk-dev
    mkdir -p "$out_dir"
    mv *.deb "$out_dir"/
}

build_arm64() {
    local host_uid host_gid
    local project_dir package_tool
    local container_build_dir="/tmp/mavsdk-build-arm64"
    local container_install_dir="/tmp/mavsdk-install-arm64"
    host_uid="$(id -u)"
    host_gid="$(id -g)"
    project_dir="$(resolve_mavsdk_project_dir)" || {
        echo "ERROR: No MAVSDK CMakeLists.txt found in $mavsdk_dir or $mavsdk_dir/cpp" >&2
        exit 1
    }
    package_tool="$(resolve_mavsdk_package_tool "$project_dir")" || {
        echo "ERROR: MAVSDK package helper not found under $mavsdk_dir" >&2
        exit 1
    }

    cd "$project_dir"
    docker run --rm \
        -v "$project_dir":/work \
        -w /work \
        -e HOST_UID="$host_uid" \
        -e HOST_GID="$host_gid" \
        -e CMAKE_VERSION="$cmake_version" \
        -e MAVSDK_PACKAGE_TOOL="$package_tool" \
        -e CMAKE_BUILD_DIR="$container_build_dir" \
        -e CMAKE_INSTALL_DIR="$container_install_dir" \
        mavsdk/mavsdk-dockcross-linux-arm64-custom \
        /bin/bash -lc '
        trap '"'"'chown -R "$HOST_UID:$HOST_GID" build install ./*.deb 2>/dev/null || true'"'"' EXIT
        rm -rf "$CMAKE_BUILD_DIR" "$CMAKE_INSTALL_DIR"
        if ! cmake --version 2>/dev/null | awk "NR==1{split(\$3,v,\".\"); exit !(v[1] > 3 || (v[1] == 3 && v[2] >= 22))}"; then
            cmake_arch=$(uname -m)
            echo "Installing CMake ${CMAKE_VERSION} for ${cmake_arch}..."
            rm -rf /tmp/cmake /tmp/cmake.tar.gz
            mkdir -p /tmp/cmake
            curl -fsSL "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-${cmake_arch}.tar.gz" -o /tmp/cmake.tar.gz
            tar -xzf /tmp/cmake.tar.gz -C /tmp/cmake --strip-components=1
            export PATH=/tmp/cmake/bin:$PATH
        fi
        cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$CMAKE_INSTALL_DIR" \
            -DBUILD_MAVSDK_SERVER=OFF -DBUILD_SHARED_LIBS=ON -DWERROR=OFF -S . -B"$CMAKE_BUILD_DIR" && \
        cmake --build "$CMAKE_BUILD_DIR" -j5 --target install && \
        "$MAVSDK_PACKAGE_TOOL" "$CMAKE_INSTALL_DIR" . arm64 libmavsdk-dev && \
        chown -R "$HOST_UID:$HOST_GID" build install ./*.deb 2>/dev/null || true
    '
    mkdir -p "$out_dir"
    mv *.deb "$out_dir"/
}

build_armv7() {
    local host_uid host_gid
    local project_dir package_tool
    local container_build_dir="/tmp/mavsdk-build-armv7"
    local container_install_dir="/tmp/mavsdk-install-armv7"
    host_uid="$(id -u)"
    host_gid="$(id -g)"
    project_dir="$(resolve_mavsdk_project_dir)" || {
        echo "ERROR: No MAVSDK CMakeLists.txt found in $mavsdk_dir or $mavsdk_dir/cpp" >&2
        exit 1
    }
    package_tool="$(resolve_mavsdk_package_tool "$project_dir")" || {
        echo "ERROR: MAVSDK package helper not found under $mavsdk_dir" >&2
        exit 1
    }

    cd "$project_dir"
    docker run --rm \
        -v "$project_dir":/work \
        -w /work \
        -e HOST_UID="$host_uid" \
        -e HOST_GID="$host_gid" \
        -e CMAKE_VERSION="$cmake_version" \
        -e MAVSDK_PACKAGE_TOOL="$package_tool" \
        -e CMAKE_BUILD_DIR="$container_build_dir" \
        -e CMAKE_INSTALL_DIR="$container_install_dir" \
        mavsdk/mavsdk-dockcross-linux-armv7-custom \
        /bin/bash -lc '
        trap '"'"'chown -R "$HOST_UID:$HOST_GID" build install ./*.deb 2>/dev/null || true'"'"' EXIT
        rm -rf "$CMAKE_BUILD_DIR" "$CMAKE_INSTALL_DIR"
        if ! cmake --version 2>/dev/null | awk "NR==1{split(\$3,v,\".\"); exit !(v[1] > 3 || (v[1] == 3 && v[2] >= 22))}"; then
            cmake_arch=$(uname -m)
            if [[ -z "${cmake_arch:-}" ]]; then
                echo "CMake >= 3.22 is required, but the container runtime architecture could not be determined." >&2
                exit 1
            fi
            echo "Installing CMake ${CMAKE_VERSION} for ${cmake_arch}..."
            rm -rf /tmp/cmake /tmp/cmake.tar.gz
            mkdir -p /tmp/cmake
            curl -fsSL "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-${cmake_arch}.tar.gz" -o /tmp/cmake.tar.gz
            tar -xzf /tmp/cmake.tar.gz -C /tmp/cmake --strip-components=1
            export PATH=/tmp/cmake/bin:$PATH
        fi
        cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$CMAKE_INSTALL_DIR" \
            -DBUILD_MAVSDK_SERVER=OFF -DBUILD_SHARED_LIBS=ON -DWERROR=OFF -S . -B"$CMAKE_BUILD_DIR" && \
        cmake --build "$CMAKE_BUILD_DIR" -j5 --target install && \
        "$MAVSDK_PACKAGE_TOOL" "$CMAKE_INSTALL_DIR" . armv7 libmavsdk-dev && \
        chown -R "$HOST_UID:$HOST_GID" build install ./*.deb 2>/dev/null || true
    '
    mkdir -p "$out_dir"
    mv *.deb "$out_dir"/
}

latest_deb() {
    find "$out_dir" -maxdepth 1 -type f -name 'libmavsdk-dev*.deb' \
        -printf '%T@ %p\n' | sort -nr | awk 'NR == 1 {print $2}'
}

mkdir -p "$out_dir"
ensure_mavsdk_checkout

info "Building MAVSDK Debian package for $arch..."
case "$arch" in
    amd64) build_amd64 ;;
    arm64) build_arm64 ;;
    armv7) build_armv7 ;;
esac

deb="$(latest_deb)"
[[ -n "$deb" ]] || { echo "ERROR: MAVSDK deb was not produced in $out_dir" >&2; exit 1; }
info "Built MAVSDK deb: $deb"
printf '%s\n' "$deb" >&3
