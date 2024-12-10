// Copyright (c) 2025 -	Dumitru Petrusca
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Config.h"
#include "W5500.h"
#include "Machine/MachineConfig.h"
#include "Channel.h"
#include "Report.h"
#include "Driver/sdspi.h"
#include <ETH.h>
#include "driver/spi_master.h"

void W5500::afterParse() {}

void W5500::init() {
    std::string msg;
    if (!_cs.defined()) {
        msg = "The CS pin is required for W5500";
    } else if (!_interrupt.defined()) {
        msg = "The interrupt pin is required for W5500";
    } else if (_ip.empty()) {
        msg = "The IP address is required for W5500";
    } else if (_gateway.empty()) {
        msg = "The geteway is required for W5500";
    } else if (_netmask.empty()) {
        msg = "The netmask is required for W5500";
    } else if (!config->_vspi->defined()) {
        msg = "VSPI is required for W5500";
    }
    if (!msg.empty()) {
        log_error(msg);
    }
    _err = msg.empty() ? ESP_OK :  ESP_ERR_INVALID_ARG;
    if (_err != ESP_OK) {
        return;
    }
    
    log_info("W5500 cs:" << _cs.name() << " interrupt:" << _interrupt.name() << 
                  " ip:" << _ip << " gateway:" << _gateway << " netmask:" << _netmask);
    _cs.setAttr(Pin::Attr::Output);
    _interrupt.setAttr(Pin::Attr::Input);
     
    _err = setupW5500();
    if (_err != ESP_OK) {
        log_error("W5500 initializatin error: " << _err);
    }
}

esp_err_t W5500::setupW5500() {
    WiFi.begin();

    tcpip_adapter_set_default_eth_handlers();

    // Initialize TCP/IP network interface
    RETURN_IF_ERROR(esp_netif_init());
    esp_netif_config_t cfg       = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t*       eth_netif = esp_netif_new(&cfg);

    esp_eth_mac_t* eth_mac = NULL;
    esp_eth_phy_t* eth_phy = NULL;

    gpio_install_isr_service(0);  // Should this be in gpio?

    spi_device_interface_config_t devcfg = {
        .command_bits = 16, // Address phase in W5500 SPI frame
        .address_bits = 8,  // Control phase in W5500 SPI frame
        .mode = 0,
        .clock_speed_hz = 12 * 1000 * 1000,
        .spics_io_num = _cs.getNative(Pin::Capabilities::Output | Pin::Capabilities::Native), // SCS
        .queue_size = 20};
    spi_device_handle_t spi_handle = NULL;
    RETURN_IF_ERROR(spi_bus_add_device(VSPI_HOST, &devcfg, &spi_handle));

    // W5500 ethernet driver uses spi driver 
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_handle);
    w5500_config.int_gpio_num       = _interrupt.getNative(Pin::Capabilities::Input | Pin::Capabilities::Native);  // INT
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    if (eth_mac == NULL) {
        log_e("esp_eth_mac_new_w5500 failed");
        return false;
    }

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.reset_gpio_num   = -1;  // TODO make this configurable
    eth_phy = esp_eth_phy_new_w5500(&phy_config);
    if (eth_phy == NULL) {
        log_e("esp_eth_phy_new_w5500 failed");
        return false;
    }

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(eth_mac, eth_phy);
    esp_eth_handle_t eth_handle = NULL;
    RETURN_IF_ERROR(esp_eth_driver_install(&eth_config, &eth_handle));

    uint8_t macArr[] = { 0x02, 0x00, 0x00, 0x12, 0x34, 0x56 };
    RETURN_IF_ERROR(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, macArr));

    // Attach Ethernet driver to TCP/IP stack 
    RETURN_IF_ERROR(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    esp_netif_ip_info_t ip_info;
    ip_info.ip.addr      = esp_ip4addr_aton(_ip.c_str());
    ip_info.gw.addr      = esp_ip4addr_aton(_gateway.c_str());
    ip_info.netmask.addr = esp_ip4addr_aton(_netmask.c_str());
    RETURN_IF_ERROR(esp_netif_dhcpc_stop(eth_netif));  // Stop DHCP client
    RETURN_IF_ERROR(esp_netif_set_ip_info(eth_netif, &ip_info));

    // Start Ethernet driver state machine 
    RETURN_IF_ERROR(esp_eth_start(eth_handle));

    return ESP_OK;
}

bool W5500::started() {
    return _err == ESP_OK;
}