#include "network_manager.h"

/* Predefined Access point names - ESP will connect to these hotspots when any one of them is present and ESP is looking to connect */
/* The Zeroth index will be set when reading credentials from NVS, when credentials are programmed using ESP's own AP + HTTP webpage, so here it is initalized as {0}*/
constexpr std::array<NETWORK::Wlan::Wlan_access_point, NETWORK::Wlan::numof_wlan_access_point_names> get_wlan_credentials() {

    std::array<NETWORK::Wlan::Wlan_access_point, NETWORK::Wlan::numof_wlan_access_point_names> wlan_credentials_list{};

    for (size_t i = 0; i < wlan_credentials_list.size(); ++i)
    {
        switch(i){
            case 0: 
                wlan_credentials_list[i] = NETWORK::Wlan::Wlan_access_point{};
                break;
            case 1:
                wlan_credentials_list[i] = NETWORK::Wlan::Wlan_access_point("florian", "pass auf die brucke auf", 0u, 0u, -99, 0u, 0u);
                break;
            case 2:
                wlan_credentials_list[i] = NETWORK::Wlan::Wlan_access_point("samir", "double caution jump", 0u, 0u, -99, 0u, 0u);
                break;
            default:
                wlan_credentials_list[i] = NETWORK::Wlan::Wlan_access_point{};
                break;
        }
    }

    return wlan_credentials_list;
}

constexpr NETWORK::Wlan::Wlan_access_point get_esp_ap_provisioning_credentials(void) {
    return NETWORK::Wlan::Wlan_access_point{"espC3_magpie", "123456789", 0u, 6u, -99, 1u, 0u};
}