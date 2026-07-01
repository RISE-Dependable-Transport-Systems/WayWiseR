#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
keyring="/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg"
sources_list="/etc/apt/sources.list.d/gazebo-stable.list"

as_root() {
    if [[ "$(id -u)" -eq 0 ]]; then
        "$@"
    else
        sudo "$@"
    fi
}

is_mandatory_package() {
    case "$1" in
        ros-"$ros_distro"-waywiser|\
        ros-"$ros_distro"-waywiser-core|\
        ros-"$ros_distro"-waywiser-description|\
        ros-"$ros_distro"-waywiser-twist-safety)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

is_waywiser_package() {
    case "$1" in
        ros-"$ros_distro"-waywiser|ros-"$ros_distro"-waywiser-*)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

is_carla_support_package() {
    case "$1" in
        ros-"$ros_distro"-carla-*|\
        ros-"$ros_distro"-ros-compatibility|\
        ros-"$ros_distro"-rqt-carla-control)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

ros_deb_to_package_name() {
    local package="$1"
    package="${package#ros-$ros_distro-}"
    printf '%s\n' "${package//-/_}"
}

select_debs_to_install() {
    local deb package ros_package result exit_status
    local -a all_debs=()
    local -a mandatory_debs=()
    local -a carla_support_debs=()
    local -a optional_packages=()
    local -a items=()
    local mandatory_lines=""
    declare -A deb_by_ros_package=()

    shopt -s nullglob
    all_debs=("$script_dir"/ros-"$ros_distro"-*.deb)
    shopt -u nullglob

    if [[ ${#all_debs[@]} -eq 0 ]]; then
        echo "No ros-${ros_distro}-*.deb packages found in $script_dir." >&2
        exit 1
    fi

    for deb in "${all_debs[@]}"; do
        package="$(dpkg-deb -f "$deb" Package)"
        if is_mandatory_package "$package"; then
            mandatory_debs+=("$deb")
            mandatory_lines+="  * $(ros_deb_to_package_name "$package")"$'\n'
        elif is_waywiser_package "$package"; then
            ros_package="$(ros_deb_to_package_name "$package")"
            optional_packages+=("$ros_package")
            deb_by_ros_package["$ros_package"]="$deb"
            items+=("$ros_package" "" "ON")
        elif is_carla_support_package "$package"; then
            carla_support_debs+=("$deb")
        fi
    done

    if [[ ! -t 2 || ${#optional_packages[@]} -eq 0 ]]; then
        printf '%s\n' "${all_debs[@]}"
        return 0
    fi

    if ! command -v dialog >/dev/null 2>&1; then
        echo "dialog not found; installing all packages." >&2
        printf '%s\n' "${all_debs[@]}"
        return 0
    fi

    local n=${#optional_packages[@]}
    local height=$(( n + 12 ))
    (( height > 34 )) && height=34
    local width=$(( $(tput cols 2>/dev/null || echo 100) - 4 ))
    (( width < 76 )) && width=76

    exit_status=0
    result=$(dialog \
        --keep-tite \
        --title "WayWiseR - Install Packages" \
        --checklist \
"Always installed (cannot be deselected):
${mandatory_lines:-  * none}

Check optional WayWiseR packages to INSTALL.
(Space = toggle  |  Arrow keys = navigate  |  Enter = confirm)" \
        "$height" "$width" "$n" \
        "${items[@]}" \
        3>&1 1>&2 2>&3) || exit_status=$?
    [[ $exit_status -eq 0 ]] || exit 1

    result="$(printf '%s\n' "$result" | tr -d '"')"
    printf '%s\n' "${mandatory_debs[@]}"
    for ros_package in "${optional_packages[@]}"; do
        [[ " $result " == *" $ros_package "* ]] || continue
        printf '%s\n' "${deb_by_ros_package[$ros_package]}"
        if [[ "$ros_package" == "waywiser_carla" ]]; then
            printf '%s\n' "${carla_support_debs[@]}"
        fi
    done
}

mapfile -t selected_debs < <(select_debs_to_install)
if [[ ${#selected_debs[@]} -eq 0 ]]; then
    echo "No packages selected for installation." >&2
    exit 1
fi

if printf '%s\n' "${selected_debs[@]}" | grep -q "/ros-${ros_distro}-waywiser-gazebo_"; then
    echo "Configuring OSRF Gazebo apt repository for WayWiseR Gazebo packages..."
    as_root install -d -m 0755 "$(dirname "$keyring")"
    if ! command -v curl >/dev/null 2>&1; then
        as_root apt-get update
        as_root apt-get install -y --no-install-recommends ca-certificates curl
    fi
    if [[ ! -f "$keyring" ]]; then
        curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
            | as_root tee "$keyring" >/dev/null
    fi

    ubuntu_codename="$(
        . /etc/os-release
        printf '%s\n' "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
    )"
    if [[ -z "$ubuntu_codename" ]]; then
        echo "Could not determine Ubuntu codename from /etc/os-release." >&2
        exit 1
    fi

    echo "deb [arch=$(dpkg --print-architecture) signed-by=$keyring] http://packages.osrfoundation.org/gazebo/ubuntu-stable $ubuntu_codename main" \
        | as_root tee "$sources_list" >/dev/null
fi

as_root apt-get update
as_root apt-get install --reinstall "${selected_debs[@]}"
