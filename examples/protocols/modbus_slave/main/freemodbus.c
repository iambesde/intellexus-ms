/* FreeModbus Slave Example ESP32

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "mbcontroller.h"       // for mbcontroller defines and api
#include "deviceparams.h"       // for device parameters structures
#include "esp_log.h"            // for log_write
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
// CHANGES FOR ETHERNET
#include "lwip/err.h"
#include "lwip/sys.h"
#include <stdio.h>
#include "esp_err.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "eth_phy/phy_lan8720.h"
#include "lwip/sockets.h"
#define DEFAULT_ETHERNET_PHY_CONFIG phy_lan8720_default_ethernet_config

#define DEVICE_IP "192.168.0.2"
#define DEVICE_GW "192.168.0.1"
#define DEVICE_NETMASK "255.255.255.0"

#define PIN_PHY_POWER CONFIG_PHY_POWER_PIN
#define PIN_SMI_MDC CONFIG_PHY_SMI_MDC_PIN
#define PIN_SMI_MDIO CONFIG_PHY_SMI_MDIO_PIN

#define MB_PORT_NUM     (502)           // Number of UART port used for Modbus connection
#define MB_DEV_ADDR     (1)           // The address of device in Modbus network
#define MB_DEV_SPEED    (115200)      // The communication speed of the UART

// Defines below are used to define register start address for each type of Modbus registers
#define MB_REG_DISCRETE_INPUT_START         (0x0000)
#define MB_REG_INPUT_START                  (0x0000)
#define MB_REG_HOLDING_START                (0x0000)
#define MB_REG_COILS_START                  (0x0000)

#define MB_PAR_INFO_GET_TOUT                (10) // Timeout for get parameter info
#define MB_CHAN_DATA_MAX_VAL                (10)
#define MB_CHAN_DAyTA_OFFSET                 (0.01f)
#define EXAMPLE_ESP_WIFI_SSID      "ESP_AP"
#define EXAMPLE_ESP_WIFI_PASS      "1234567890"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10
static const char *TAG = "MODBUS_SLAVE_APP";
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_eth_event_group;

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int ETH_CONNECTED_BIT = BIT0;

static int s_retry_num = 0;
mb_param_info_t reg_info; // keeps the Modbus registers access information
mb_communication_info_t comm_info; // Modbus communication parameters
mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
eMBErrorCode eStatus;
nvs_handle my_handle;

/**
 * @brief gpio specific init
 *
 * @note RMII data pins are fixed in esp32:
 * TXD0 <=> GPIO19
 * TXD1 <=> GPIO22
 * TX_EN <=> GPIO21
 * RXD0 <=> GPIO25
 * RXD1 <=> GPIO26
 * CLK <=> GPIO0
 *
 */
static void eth_gpio_config_rmii(void)
{
    phy_rmii_configure_data_interface_pins();
    phy_rmii_smi_configure_pins(PIN_SMI_MDC, PIN_SMI_MDIO);
}
#ifdef CONFIG_PHY_USE_POWER_PIN
/**
 * @brief re-define power enable func for phy
 *
 * @param enable true to enable, false to disable
 *
 * @note This function replaces the default PHY power on/off function.
 * If this GPIO is not connected on your device (and PHY is always powered),
 * you can use the default PHY-specific power on/off function.
 */
static void phy_device_power_enable_via_gpio(bool enable)
{
    assert(DEFAULT_ETHERNET_PHY_CONFIG.phy_power_enable);

    if (!enable) {
        DEFAULT_ETHERNET_PHY_CONFIG.phy_power_enable(false);
    }

    gpio_pad_select_gpio(PIN_PHY_POWER);
    gpio_set_direction(PIN_PHY_POWER, GPIO_MODE_OUTPUT);
    if (enable == true) {
        gpio_set_level(PIN_PHY_POWER, 1);
        ESP_LOGI(TAG, "Power On Ethernet PHY");
    } else {
        gpio_set_level(PIN_PHY_POWER, 0);
        ESP_LOGI(TAG, "Power Off Ethernet PHY");
    }

    vTaskDelay(1); // Allow the power up/down to take effect, min 300us

    if (enable) {
        /* call the default PHY-specific power on function */
        DEFAULT_ETHERNET_PHY_CONFIG.phy_power_enable(true);
    }
}
#endif
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    tcpip_adapter_ip_info_t ip;
    switch(event->event_id) {
   /* case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        {
                esp_wifi_connect();
            if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                s_retry_num++;
                ESP_LOGI(TAG,"retry to connect to the AP");
            }
            ESP_LOGI(TAG,"connect to the AP fail\n");
            break;
        }*/
    	   case SYSTEM_EVENT_ETH_CONNECTED:
               ESP_LOGI(TAG, "Ethernet Link Up");
               break;
           case SYSTEM_EVENT_ETH_DISCONNECTED:
               ESP_LOGI(TAG, "Ethernet Link Down");
				xEventGroupClearBits(s_eth_event_group, ETH_CONNECTED_BIT);
               break;
           case SYSTEM_EVENT_ETH_START:
               ESP_LOGI(TAG, "Ethernet Started");
               break;
           case SYSTEM_EVENT_ETH_GOT_IP:
        	   memset(&ip, 0, sizeof(tcpip_adapter_ip_info_t));
               ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(ESP_IF_ETH, &ip));
               ESP_LOGI(TAG, "Ethernet Got IP Addr");
               ESP_LOGI(TAG, "~~~~~~~~~~~");
               ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip.ip));
               ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip.netmask));
               ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip.gw));
               ESP_LOGI(TAG, "~~~~~~~~~~~");
	       xEventGroupSetBits(s_eth_event_group, ETH_CONNECTED_BIT);
               break;
           case SYSTEM_EVENT_ETH_STOP:
               ESP_LOGI(TAG, "Ethernet Stopped");
               break;
    default:
        break;
    }
    return ESP_OK;
}

// Set register values into known state
static void setup_reg_data()
{
    // Define initial state of parameters
    discrete_reg_params.discrete_input1 = 1;
    discrete_reg_params.discrete_input3 = 1;
    discrete_reg_params.discrete_input5 = 1;
    discrete_reg_params.discrete_input7 = 1;

    holding_reg_params.data_chan[0] = 1.00;
    holding_reg_params.data_chan[1] = 2.00;
    holding_reg_params.data_chan[2] = 3.00;
    holding_reg_params.data_chan[3] = 4.00;
    holding_reg_params.data_chan[4] = 5.00;
    holding_reg_params.data_chan[5] = 6.00;
    holding_reg_params.data_chan[6] = 7.00;
    holding_reg_params.data_chan[7] = 8.00;
    holding_reg_params.data_chan[8] = 9.00;
    holding_reg_params.data_chan[9] = 10.00;

    coil_reg_params.coil0 = 1;
    coil_reg_params.coil2 = 1;
    coil_reg_params.coil4 = 1;
    coil_reg_params.coil6 = 1;
    coil_reg_params.coil7 = 1;

    input_reg_params.data_chan0 = 1.34;
    input_reg_params.data_chan1 = 2.56;
    input_reg_params.data_chan2 = 3.78;
    input_reg_params.data_chan3 = 4.90;
}
static void vEthernetInit(void)
{
		tcpip_adapter_ip_info_t ipInfo;
		s_eth_event_group = xEventGroupCreate();

		tcpip_adapter_init();
		ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

		eth_config_t config = DEFAULT_ETHERNET_PHY_CONFIG;
	    config.phy_addr = PHY1;
	    config.gpio_config = eth_gpio_config_rmii;
	    config.tcpip_input = tcpip_adapter_eth_input;
	    config.clock_mode = CONFIG_PHY_CLOCK_MODE;
	#ifdef CONFIG_PHY_USE_POWER_PIN
	    /* Replace the default 'power enable' function with an example-specific one
	     that toggles a power GPIO. */
	    config.phy_power_enable = phy_device_power_enable_via_gpio;
	#endif
		
	    ESP_ERROR_CHECK(esp_eth_init(&config));
	    ESP_ERROR_CHECK(esp_eth_enable()) ;
	    tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_ETH); // Don't run a DHCP client
	    
	                            inet_pton(AF_INET, DEVICE_IP, &ipInfo.ip);
	                            inet_pton(AF_INET, DEVICE_GW, &ipInfo.gw);
	                            inet_pton(AF_INET, DEVICE_NETMASK, &ipInfo.netmask);
	                            tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_ETH, &ipInfo);
}

void modbus_init(void)
{

    mbcontroller_init(); // Initialization of Modbus controller
    // Setup communication parameters and start stack
        comm_info.mode = MB_MODE_RTU;
        comm_info.slave_addr = MB_DEV_ADDR;
        comm_info.port = MB_PORT_NUM;
        comm_info.baudrate = MB_DEV_SPEED;
        comm_info.parity = MB_PARITY_NONE;
        ESP_ERROR_CHECK(mbcontroller_setup(comm_info));
        // The code below initializes Modbus register area descriptors
            // for Modbus Holding Registers, Input Registers, Coils and Discrete Inputs
            // Initialization should be done for each supported Modbus register area according to register map.
            // When external master trying to access the register in the area that is not initialized
            // by mbcontroller_set_descriptor() API call then Modbus stack
            // will send exception response for this register area.
            reg_area.type = MB_PARAM_HOLDING; // Set type of register area
            reg_area.start_offset = MB_REG_HOLDING_START; // Offset of register area in Modbus protocol
            reg_area.address = (void*)&holding_reg_params; // Set pointer to storage instance
            reg_area.size = sizeof(holding_reg_params); // Set the size of register storage instance
            ESP_ERROR_CHECK(mbcontroller_set_descriptor(reg_area));

            // Initialization of Input Registers area
            reg_area.type = MB_PARAM_INPUT;
            reg_area.start_offset = MB_REG_INPUT_START;
            reg_area.address = (void*)&input_reg_params;
            reg_area.size = sizeof(input_reg_params);
            ESP_ERROR_CHECK(mbcontroller_set_descriptor(reg_area));

            // Initialization of Coils register area
            reg_area.type = MB_PARAM_COIL;
            reg_area.start_offset = MB_REG_COILS_START;
            reg_area.address = (void*)&coil_reg_params;
            reg_area.size = sizeof(coil_reg_params);
            ESP_ERROR_CHECK(mbcontroller_set_descriptor(reg_area));

            // Initialization of Discrete Inputs register area
            reg_area.type = MB_PARAM_DISCRETE;
            reg_area.start_offset = MB_REG_DISCRETE_INPUT_START;
            reg_area.address = (void*)&discrete_reg_params;
            reg_area.size = sizeof(discrete_reg_params);
            ESP_ERROR_CHECK(mbcontroller_set_descriptor(reg_area));

            setup_reg_data(); // Set values into known state

            // Starts of modbus controller and stack
            ESP_ERROR_CHECK(mbcontroller_start());

}
void nvs_init(void)
{
	printf("NVS Demo, esp32-tutorial\n\n");

	        // initialize NVS flash
	esp_err_t err = nvs_flash_init();

	        // if it is invalid, try to erase it
	if (err == ESP_ERR_NVS_NO_FREE_PAGES) {

	       printf("Got NO_FREE_PAGES error, trying to erase the partition...\n");

	                // find the NVS partition
	       const esp_partition_t* nvs_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
	                if(!nvs_partition) {

	                        printf("FATAL ERROR: No NVS partition found\n");
	                        while(1) vTaskDelay(10 / portTICK_PERIOD_MS);
	                }

	                // erase the partition
	        err = (esp_partition_erase_range(nvs_partition, 0, nvs_partition->size));
	                if(err != ESP_OK) {
	                        printf("FATAL ERROR: Unable to erase the partition\n");
	                        while(1) vTaskDelay(10 / portTICK_PERIOD_MS);
	                }
	                printf("Partition erased!\n");

	                // now try to initialize it again
	                err = nvs_flash_init();
	                if(err != ESP_OK) {
	                                        printf("FATAL ERROR: Unable to initialize NVS\n");
	                                        while(1) vTaskDelay(10 / portTICK_PERIOD_MS);
	                                }
          }
          printf("NVS init OK!\n");
          // open the partition in RW mode
           err = nvs_open("storage", NVS_READWRITE, &my_handle);
           if (err != ESP_OK) {
        	   	   	   	   	   	   printf("FATAL ERROR: Unable to open NVS\n");
	                                while(1) vTaskDelay(10 / portTICK_PERIOD_MS);
	                        }
	       printf("NVS open OK\n");



}
void app_main()
{

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
            // initialize NVS

    tcpip_adapter_init();

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    nvs_init();
   vEthernetInit();
   int bits = xEventGroupWaitBits(s_eth_event_group, ETH_CONNECTED_BIT, 0, 1, portMAX_DELAY);
   printf("MODBUSInit\n");
   modbus_init();

    



    // The cycle below will be terminated when parameter holdingRegParams.dataChan0
    // incremented each access cycle reaches the CHAN_DATA_MAX_VAL value.
    for(;;){
        // Check for read/write events of Modbus master for certain events
        mb_event_group_t event = mbcontroller_check_event((MB_EVENT_HOLDING_REG_WR
                                                        | MB_EVENT_INPUT_REG_RD
                                                        | MB_EVENT_HOLDING_REG_RD
                                                        | MB_EVENT_DISCRETE_RD));
        // Filter events and process them accordingly
        if((event & MB_EVENT_HOLDING_REG_WR) || (event & MB_EVENT_HOLDING_REG_RD)) {
            // Get parameter information from parameter queue
            ESP_ERROR_CHECK(mbcontroller_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            printf("HOLDING READ/WRITE: time_stamp(us):%u, mb_addr:%u, type:%u, st_address:0x%.4x, size:%u\r\n",
                    (uint32_t)reg_info.time_stamp,
                    (uint32_t)reg_info.mb_offset,
                    (uint32_t)reg_info.type,
                    (uint32_t)reg_info.address,
                    (uint32_t)reg_info.size);
//            if (reg_info.address == (uint8_t*)&holding_reg_params.data_chan0)
//            {
//                holding_reg_params.data_chan0 += MB_CHAN_DATA_OFFSET;
//            }
            for(int i=0;i<10;i++)
            {
            	char parameter1[2] ;
            	char * key =parameter1;
            	itoa(i,parameter1,10);

            	char parameter2[15] ;
            	char *value =parameter2;
		sprintf(parameter2,"%f",holding_reg_params.data_chan[i]);
//		gcvt(holding_reg_params.data_chan[i],15,parameter2);

                    esp_err_t err = nvs_set_str(my_handle, key, value);
                            if(err != ESP_OK) {
                                    printf("\nError in nvs_set_str! (%04X)\n", err);
                                    return;
                            }
                            err = nvs_commit(my_handle);
                            if(err != ESP_OK) {
                                    printf("\nError in commit! (%04X)\n", err);
                                    return;
                            }
                            printf("\nValue %s stored in NVS with key %s\n", parameter2, parameter1);
            }

        } else if (event & MB_EVENT_INPUT_REG_RD) {
            ESP_ERROR_CHECK(mbcontroller_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            printf("INPUT READ: time_stamp(us):%u, mb_addr:%u, type:%u, st_address:0x%.4x, size:%u\r\n",
                    (uint32_t)reg_info.time_stamp,
                    (uint32_t)reg_info.mb_offset,
                    (uint32_t)reg_info.type,
                    (uint32_t)reg_info.address,
                    (uint32_t)reg_info.size);
        } else if (event & MB_EVENT_DISCRETE_RD) {
            ESP_ERROR_CHECK(mbcontroller_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            printf("DISCRETE READ: time_stamp(us):%u, mb_addr:%u, type:%u, st_address:0x%.4x, size:%u\r\n",
                                (uint32_t)reg_info.time_stamp,
                                (uint32_t)reg_info.mb_offset,
                                (uint32_t)reg_info.type,
                                (uint32_t)reg_info.address,
                                (uint32_t)reg_info.size);
        } else if (event & MB_EVENT_COILS_RD) {
            ESP_ERROR_CHECK(mbcontroller_get_param_info(&reg_info, MB_PAR_INFO_GET_TOUT));
            printf("COILS READ: time_stamp(us):%u, mb_addr:%u, type:%u, st_address:0x%.4x, size:%u\r\n",
                                (uint32_t)reg_info.time_stamp,
                                (uint32_t)reg_info.mb_offset,
                                (uint32_t)reg_info.type,
                                (uint32_t)reg_info.address,
                                (uint32_t)reg_info.size);
        }
    }
    // Destroy of Modbus controller once get maximum value of data_chan0
    printf("Modbus controller destroyed.");
    ESP_ERROR_CHECK(mbcontroller_destroy());
}
