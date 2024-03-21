#!/bin/bash

# Function to display usage information
display_usage() {
    echo "Usage: $(basename "$0") [-h] [-e] [-r] [-su] [-s server_ip] [-c client_ip] [-d domain_id]"
    echo "Options:"
    echo "  -h: Display this help message"
    echo "  -e: Edit default config file"
    echo "  -r: Configures using remote client config file"
    echo "  -su: Configures as super client"
    echo "  -s server_ip: Server IP"
    echo "  -c client_ip: Client IP"
    echo "  -d domain_id: Domain ID (must be >= 0 and < 200)"
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
local_client_config_file="local_client.xml"
remote_client_config_file="remote_client.xml"
tmp_dir="/tmp/waywiser/discovery/"
mkdir -p $tmp_dir

edit_config=false
remote_client=false
is_super_client=0
server_ip=""
client_ip=""
domain_id_input=-1

# Parse options
while getopts "hers:c:d:" opt; do
    case $opt in
    h)
        display_usage
        exit 0
        ;;
    e) edit_config=true ;;
    r) remote_client=true ;;
    s)
        if [[ "$OPTARG" == "u" ]]; then
            is_super_client=1
        else
            server_ip="$OPTARG"
        fi
        ;;
    c) client_ip="$OPTARG" ;;
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

if [ "$is_super_client" -eq 1 ]; then
    discovery_protocol="SUPER_CLIENT"
else
    discovery_protocol="CLIENT"
fi

if $remote_client; then
    config_file=$remote_client_config_file
    config_fullfilepath=$(get_full_file_path $remote_client_config_file)

    configured_server_ip=$(grep -oP -m 1 '(?<=<address _marker="server">)[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+(?=</address>)' $config_fullfilepath)

    if [ -z "$server_ip" ]; then
        server_ip=$configured_server_ip
    fi

    configured_client_ip=$(grep -oP -m 1 '(?<=<address _marker="client">)[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+(?=</address>)' $config_fullfilepath)
    if [ -z "$client_ip" ]; then
        client_ip=$configured_client_ip
    fi

    configured_domain_id=$(grep -oP -m 1 '(?<=<domainId>)[0-9]+(?=</domainId>)' $config_fullfilepath)
    if (($domain_id_input < 0)); then
        domain_id=$configured_domain_id
    else
        domain_id=$domain_id_input
    fi

    configured_discovery_protocol=$(grep -oP '(?<=<discoveryProtocol>).*?(?=</discoveryProtocol>)' $config_fullfilepath)

    # Check if server_ip and client_ip are valid IP addresses
    if is_valid_ip "$server_ip" && is_valid_ip "$client_ip"; then
        # Check if the system has the specified client_ip address in any of its network interfaces
        if has_ip_in_interfaces "$client_ip"; then
            if [ "$configured_server_ip" != "$server_ip" ] || [ "$configured_client_ip" != "$client_ip" ] || [ "$configured_domain_id" != "$domain_id" ] || [ "$configured_discovery_protocol" != "$discovery_protocol" ]; then
                if ! $edit_config; then
                    # Copy the configuration file to the tmp_dir and replace the server_ip and client_ip in the file
                    tmp_config_fullfilepath="$tmp_dir${config_file%.xml}_${server_ip//./}_${client_ip//./}_${domain_id}_$is_super_client.xml"
                    cp $config_fullfilepath $tmp_config_fullfilepath

                    config_fullfilepath=$tmp_config_fullfilepath
                fi

                sed -i "s|<address _marker=\"server\">$configured_server_ip</address>|<address _marker=\"server\">$server_ip</address>|g" "$config_fullfilepath"
                sed -i "s|<address _marker=\"client\">$configured_client_ip</address>|<address _marker=\"client\">$client_ip</address>|g" "$config_fullfilepath"
                sed -i "s|<domainId>$configured_domain_id</domainId>|<domainId>$domain_id</domainId>|g" "$config_fullfilepath"
                sed -i "s|<discoveryProtocol>$configured_discovery_protocol</discoveryProtocol>|<discoveryProtocol>$discovery_protocol</discoveryProtocol>|g" $config_fullfilepath

                echo "Edited the config_file: $config_fullfilepath to access the remote server $server_ip from the client $client_ip with domain id $domain_id and $discovery_protocol protocol."
            fi
        else
            echo "Error: The system does not have the client ip $client_ip in any of its interfaces."
            exit 1
        fi
    else
        echo "Error: Make sure server_ip $server_ip and client_ip $client_ip are valid ip addresses."
        exit 1
    fi

    export ROS_DISCOVERY_SERVER=";UDPv4:[$server_ip]:11812"
else
    config_file=$local_client_config_file
    config_fullfilepath=$(get_full_file_path $local_client_config_file)

    configured_domain_id=$(grep -oP -m 1 '(?<=<domainId>)[0-9]+(?=</domainId>)' $config_fullfilepath)
    if (($domain_id_input < 0)); then
        domain_id=$configured_domain_id
    else
        domain_id=$domain_id_input
    fi

    configured_discovery_protocol=$(grep -oP '(?<=<discoveryProtocol>).*?(?=</discoveryProtocol>)' $config_fullfilepath)

    if [ "$configured_domain_id" != "$domain_id" ] || [ "$configured_discovery_protocol" != "$discovery_protocol" ]; then
        if ! $edit_config; then
            # Copy the configuration file to the tmp_dir and replace the server_ip and client_ip in the file
            tmp_config_fullfilepath="$tmp_dir${config_file%.xml}_${domain_id}_$is_super_client.xml"
            cp $config_fullfilepath $tmp_config_fullfilepath

            config_fullfilepath=$tmp_config_fullfilepath
        fi

        sed -i "s|<domainId>$configured_domain_id</domainId>|<domainId>$domain_id</domainId>|g" "$config_fullfilepath"
        sed -i "s|<discoveryProtocol>$configured_discovery_protocol</discoveryProtocol>|<discoveryProtocol>$discovery_protocol</discoveryProtocol>|g" $config_fullfilepath

        echo "Edited the config_file: $config_fullfilepath to access the local server with domain id $domain_id and $discovery_protocol protocol."
    fi

    export ROS_DISCOVERY_SERVER="UDPv4:[127.0.0.1]:11811;UDPv4:[127.0.0.1]:11812"
fi

export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export FASTRTPS_DEFAULT_PROFILES_FILE=$config_fullfilepath
export ROS_SUPER_CLIENT=$is_super_client
export ROS_DOMAIN_ID=$domain_id

echo "### Client is configured ###"
echo "  RMW_FASTRTPS_USE_QOS_FROM_XML=\"$RMW_FASTRTPS_USE_QOS_FROM_XML\""
echo "  ROS_DISCOVERY_SERVER=\"$ROS_DISCOVERY_SERVER\""
echo "  FASTRTPS_DEFAULT_PROFILES_FILE=\"$FASTRTPS_DEFAULT_PROFILES_FILE\""
echo "  ROS_DOMAIN_ID=\"$ROS_DOMAIN_ID\""
echo "  ROS_SUPER_CLIENT=\"$ROS_SUPER_CLIENT\""
