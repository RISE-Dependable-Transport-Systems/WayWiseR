#!/bin/bash

# Function to display usage information
display_usage() {
    echo "Usage: $(basename "$0") [-h] [-e] [-r] [-s server_ip] [-d domain_id]"
    echo "Options:"
    echo "  -h: Display this help message"
    echo "  -e: Edit default config file"
    echo "  -r: Start remote server (along with local server)"
    echo "  -s server_ip: Server IP"
    echo "  -d domain_id: Domain ID (must be >= 0 and < 200)"
}

# Function to start server and capture output
server_setup() {
    local server_id=$1
    local file_path=$(get_full_file_path $2)

    # Start server and capture output
    fastdds discovery --server-id $server_id -x $file_path | while IFS= read -r line; do
        echo "[server $server_id] $line"
    done
}

# Function to get full file path
get_full_file_path() {
    local file_path=$1

    if [ ! -f "$file_path" ]; then
        # Find config fullfilepath
        if [ -d "src" ]; then
            file_path="$PWD/src/WayWiseR/waywiser/discovery/$file_path"
        elif [ -d "waywiser" ]; then
            file_path="$PWD/waywiser/discovery/$file_path"
        else
            echo "Error: Unable to find file $file_path"
            exit 1
        fi
    fi

    echo "$file_path"
}

# Function to check if input contains a valid IP address
is_valid_ip() {
    local ip_pattern='^([0-9]{1,3}\.){3}[0-9]{1,3}$'
    if [[ $1 =~ $ip_pattern ]]; then
        return 0 # IP address is valid
    else
        return 1 # IP address is not valid
    fi
}

# Function to check if the system has the specified IP address in any of its interfaces
has_ip_in_interfaces() {
    local ip=$1
    local interfaces=$(ip addr | grep -oP '(?<=inet )([0-9]{1,3}\.){3}[0-9]{1,3}')
    for interface_ip in $interfaces; do
        if [[ $interface_ip == $ip ]]; then
            return 0 # IP address is found in an interface
        fi
    done
    return 1 # IP address is not found in any interface
}

########################################################################

# Default values
local_server_config_file="local_server.xml"
remote_server_config_file="remote_server.xml"
tmp_dir="$HOME/.waywiser/discovery/"
mkdir -p $tmp_dir

edit_config=false
start_remote=false
server_ip=""
domain_id_input=-1

# Parse options
while getopts "hers:d:" opt; do
    case $opt in
    h)
        display_usage
        exit 0
        ;;
    e) edit_config=true ;;
    r) start_remote=true ;;
    s) server_ip="$OPTARG" ;;
    d) # Check if domain_id is valid
        if [[ $OPTARG =~ ^[0-9]+$ ]] && ((OPTARG >= 0 && OPTARG < 200)); then
            domain_id_input="$OPTARG"
        else
            echo "Error: Invalid domain_id. It must be an integer >= 0 and < 200." >&2
            display_usage
            exit 1
        fi
        ;;
    \?)
        echo "Invalid option: -$OPTARG" >&2
        display_usage
        exit 1
        ;;
    :)
        echo "Option -$OPTARG requires an argument." >&2
        display_usage
        exit 1
        ;;
    esac
done
shift $((OPTIND - 1))

# Clean shm zombies
fastdds shm clean >/dev/null 2>&1

# Configure local server
server_id=0
config_fullfilepath=$(get_full_file_path $local_server_config_file)
configured_domain_id=$(grep -oP -m 1 '(?<=<domainId>)[0-9]+(?=</domainId>)' $config_fullfilepath)
if (($domain_id_input < 0)); then
    domain_id=$configured_domain_id
else
    domain_id=$domain_id_input
fi

if [ "$configured_domain_id" -ne "$domain_id" ]; then
    if ! $edit_config; then
        # Copy the configuration file to the tmp_dir and replace the domain_id in the file
        tmp_config_fullfilepath="$tmp_dir${local_server_config_file%.xml}_${domain_id}.xml"
        cp $config_fullfilepath $tmp_config_fullfilepath

        config_fullfilepath=$tmp_config_fullfilepath

        echo "[server $server_id] Using the config_file: $config_fullfilepath to create the local server with domain id $domain_id."
    else
        echo "[server $server_id] Edited the default config_file: $config_fullfilepath to create the local server with domain id $domain_id."
    fi

    sed -i "s|<domainId>$configured_domain_id</domainId>|<domainId>$domain_id</domainId>|g" "$config_fullfilepath"
fi

# Start local server
server_setup $server_id $config_fullfilepath &

# Configure remote server if requested
if $start_remote; then
    server_id=1

    config_fullfilepath=$(get_full_file_path $remote_server_config_file)
    configured_server_ip=$(grep -oP -m 1 '(?<=<address _marker="server">)[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+(?=</address>)' $config_fullfilepath)

    if [ -z "$server_ip" ]; then
        server_ip=$configured_server_ip
    fi

    configured_domain_id=$(grep -oP -m 1 '(?<=<domainId>)[0-9]+(?=</domainId>)' $config_fullfilepath)
    if (($domain_id_input < 0)); then
        domain_id=$configured_domain_id
    else
        domain_id=$domain_id_input
    fi

    # Check if server_ip is a valid IP address
    if is_valid_ip "$server_ip"; then
        # Check if the system has the specified IP address in any of its network interfaces
        if has_ip_in_interfaces "$server_ip"; then
            if [ "$configured_server_ip" != "$server_ip" ] || [ "$configured_domain_id" != "$domain_id" ]; then
                if ! $edit_config; then
                    # Copy the configuration file to the tmp_dir and replace the server_ip in the file
                    tmp_config_fullfilepath="$tmp_dir${remote_server_config_file%.xml}_${server_ip//./}_${domain_id}.xml"
                    cp $config_fullfilepath $tmp_config_fullfilepath

                    config_fullfilepath=$tmp_config_fullfilepath

                    echo "[server $server_id] Using the config_file: $config_fullfilepath to create the remote server $server_ip with domain id $domain_id."
                else
                    echo "[server $server_id] Edited the default config_file: $config_fullfilepath to create the remote server $server_ip with domain id $domain_id."
                fi

                sed -i "s|<address _marker=\"server\">$configured_server_ip</address>|<address _marker=\"server\">$server_ip</address>|g" "$config_fullfilepath"
                sed -i "s|<domainId>$configured_domain_id</domainId>|<domainId>$domain_id</domainId>|g" "$config_fullfilepath"

            fi

            # Start remote server
            server_setup $server_id $config_fullfilepath &
        else
            echo "[server $server_id] Error: The system does not have the server ip $server_ip in any of its interfaces."
        fi
    else
        echo "[server $server_id] Error: $server_ip is not a valid ip address."
    fi
fi

# Echo domain id
echo "Configured DOMAIN_ID : $domain_id"
########################################################################

# Wait for background processes to finish
wait
