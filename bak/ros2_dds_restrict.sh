ros2_dds_restrict(){
    # restrict Cyclone DDS to this network interface
    if [[ $# -eq 0 ]]; then
        echo "ros2_dds_restrict: give a network interface"
        return
    fi
    if [[ ! $2 ]]; then
        echo 'ros2_dds_restrict: give second input flag'
        return
    fi

    # auto-detect if basic name
    local interface=$1
    if [[ $1 == "WIFI" ]]; then
        local interface=$(for dev in /sys/class/net/*; do [ -e "$dev"/wireless ] && echo ${dev##*/}; done)

        # Trim DOWN and docker0
        local interface=$(ip link | awk -F: '$0 !~ "lo|vir|wl|^[^0-9]"{print $2;getline}')
        local interface=$(for dev in $interface; do
            [ ! -e /sys/class/net/"$dev"/wireless ] &&
            [[ ! "$dev" =~ docker ]] &&
            [ "$(cat /sys/class/net/"$dev"/operstate 2>/dev/null)" = "up" ] &&
            echo ${dev##*/}
        done)
    fi
    if [[ $1 == "ETH" ]]; then
        local interface=$(ip link | awk -F: '$0 !~ "lo|vir|wl|^[^0-9]"{print $2;getline}')
        local interface=$(for dev in $interface; do [ ! -e /sys/class/net/"$dev"/wireless ] && echo ${dev##*/}; done)

        # Trim DOWN and docker0
        local interface=$(ip link | awk -F: '$0 !~ "lo|vir|wl|^[^0-9]"{print $2;getline}')
        local interface=$(for dev in $interface; do
            [ ! -e /sys/class/net/"$dev"/wireless ] &&
            [[ ! "$dev" =~ docker ]] &&
            [ "$(cat /sys/class/net/"$dev"/operstate 2>/dev/null)" = "up" ] &&
            echo ${dev##*/}
        done)
    fi
    if [[ $1 == "lo" ]]; then
        export ROS_LOCALHOST_ONLY=1
        unset ROS_DOMAIN_ID
        unset FASTRTPS_DEFAULT_PROFILES_FILE
        # https://answers.ros.org/question/365051/using-ros2-offline-ros_localhost_only1/
        export CYCLONEDDS_URI='<General>
            <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
            <Peers>
                <Peer address="localhost"/>
            </Peers>
        </Discovery>'
        return
    fi

    # Cyclone DDS https://dds-demonstrators.readthedocs.io/en/latest/Teams/1.Hurricane/setupCycloneDDS.html
    if [[ $2 == 'SET' ]]; then
        export CYCLONEDDS_URI="<General><NetworkInterfaceAddress>$interface"
        echo "Set CYCLONEDDS_URI to ${CYCLONEDDS_URI}"

        unset ROS_LOCALHOST_ONLY  # probably do not want to limit to localhost
        echo "Unset ROS_LOCALHOST_ONLY"
    else
        echo "Found interfaces: ${interface}"
    fi

    #     # Fast-DDS https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/whitelist.html
    #     # needs actual ip for this interface
    #     ipinet="$(ip a s $interface | egrep -o 'inet [0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}')"
    #     echo "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
    #     <profiles xmlns=\"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles\">
    #         <transport_descriptors>
    #             <transport_descriptor>
    #                 <transport_id>CustomUDPTransport</transport_id>
    #                 <type>UDPv4</type>
    #                 <interfaceWhiteList>
    #                     <address>${ipinet##inet }</address>
    #                 </interfaceWhiteList>
    #             </transport_descriptor>
    #
    #             <transport_descriptor>
    #                 <transport_id>CustomTcpTransport</transport_id>
    #                 <type>TCPv4</type>
    #                 <interfaceWhiteList>
    #                     <address>${ipinet##inet }</address>
    #                 </interfaceWhiteList>
    #             </transport_descriptor>
    #
    #         </transport_descriptors>
    #
    #         <participant profile_name=\"CustomUDPTransportParticipant\">
    #             <rtps>
    #                 <userTransports>
    #                     <transport_id>CustomUDPTransport</transport_id>
    #                 </userTransports>
    #             </rtps>
    #         </participant>
    #
    #         <participant profile_name=\"CustomTcpTransportParticipant\">
    #             <rtps>
    #                 <userTransports>
    #                     <transport_id>CustomTcpTransport</transport_id>
    #                 </userTransports>
    #             </rtps>
    #         </participant>
    #     </profiles>" > /tmp/fastrtps_interface_restriction.xml
    #     # tell where to look
    #     export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastrtps_interface_restriction.xml
}
