/* Simple WiFi Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"

#include <sys/socket.h>

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      ""
#define EXAMPLE_ESP_WIFI_PASS      ""
#define EXAMPLE_MAX_STA_CONN       CONFIG_MAX_STA_CONN

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

static const char *TAG = "simple wifi";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

#if 0
void wifi_init_softap()
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}
#endif

void wifi_init_sta()
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

struct netconn *pconn;
vprintf_like_t vprintf_old;

int vprintf_net(const char *fmt, va_list ap)
{
	static char buf[512];
	if (pconn) {
		int n = vsnprintf(buf, 511, fmt, ap);
		if (n > 0) {
			if (netconn_write(pconn, buf, n, NETCONN_COPY) != ESP_OK) {
				netconn_close(pconn);
				netconn_delete(pconn);
				pconn = NULL;
			}
		}
		return n;
	} else {
		return vprintf_old(fmt, ap);
	}
}

static void logger_thread(void *p)
{
	err_t err;
	struct netconn *sconn;
	struct netconn *conn;

	sconn = netconn_new(NETCONN_TCP);
	if (sconn == NULL)
		return;

	err = netconn_bind(sconn, NULL, 8888);
	if (err != ERR_OK)
		return;

	err = netconn_listen(sconn);
	if (err != ERR_OK)
		return;

	vprintf_old = esp_log_set_vprintf(vprintf_net);

	for (;;) {
		err = netconn_accept(sconn, &conn);
		if (err == ERR_OK) {
			if (pconn)
				netconn_delete(pconn);
			pconn = conn;
		}
	}
}


static void __attribute__((noreturn)) task_fatal_error()
{
	ESP_LOGE(TAG, "Exiting task due to fatal error...");
	(void)vTaskDelete(NULL);

	while (1) {
		;
	}
}

#define BUFFSIZE 1024
#define TEXT_BUFFSIZE 1024
static char text[TEXT_BUFFSIZE];
static char ota_write_data[BUFFSIZE + 1] = { 0 };

static void ota_thread(void *pvParameter)
{
	int binary_file_length = 0;
	esp_err_t err;
	esp_ota_handle_t update_handle = 0 ;
	const esp_partition_t *update_partition = NULL;

	ESP_LOGI(TAG, "Starting OTA example...");

	const esp_partition_t *configured = esp_ota_get_boot_partition();
	const esp_partition_t *running = esp_ota_get_running_partition();

	if (configured != running) {
		ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
			 configured->address, running->address);
		ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
	}
	ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
		 running->type, running->subtype, running->address);

	struct sockaddr_in sock_info;

	int sfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sfd == -1) {
		ESP_LOGE(TAG, "Create socket failed!");
		return false;
	}

	memset(&sock_info, 0, sizeof(struct sockaddr_in));
	sock_info.sin_family = AF_INET;
	sock_info.sin_addr.s_addr = htonl(INADDR_ANY);
	sock_info.sin_port = htons(9999);

	if(bind(sfd, (struct sockaddr *)&sock_info, sizeof(sock_info)) < 0)
		task_fatal_error();

	if(listen(sfd, 10) < 0)
		task_fatal_error();

	int socket_id = accept(sfd, NULL, 0);

	send(socket_id, "hello world", 11, 0);
	update_partition = esp_ota_get_next_update_partition(NULL);
	ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
		 update_partition->subtype, update_partition->address);
	assert(update_partition != NULL);

	err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
		task_fatal_error();
	}
	ESP_LOGI(TAG, "esp_ota_begin succeeded");

	bool flag = true;
	while (flag) {
		memset(text, 0, TEXT_BUFFSIZE);
		memset(ota_write_data, 0, BUFFSIZE);
		int buff_len = recv(socket_id, text, TEXT_BUFFSIZE, 0);
		if (buff_len < 0) { /*receive error*/
			ESP_LOGE(TAG, "Error: receive data error! errno=%d", errno);
			task_fatal_error();
		} else if (buff_len > 0) { /*deal with response body*/
			memcpy(ota_write_data, text, buff_len);
			err = esp_ota_write( update_handle, (const void *)ota_write_data, buff_len);
			if (err != ESP_OK) {
				ESP_LOGE(TAG, "Error: esp_ota_write failed (%s)!", esp_err_to_name(err));
				task_fatal_error();
			}
			binary_file_length += buff_len;
			ESP_LOGI(TAG, "Have written image length %d", binary_file_length);
		} else if (buff_len == 0) {  /*packet over*/
			flag = false;
			ESP_LOGI(TAG, "Connection closed, all packets received");
			close(socket_id);
		} else {
			ESP_LOGE(TAG, "Unexpected recv result");
		}
	}

	ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);

	if (esp_ota_end(update_handle) != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_end failed!");
		task_fatal_error();
	}
	err = esp_ota_set_boot_partition(update_partition);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
		task_fatal_error();
	}
	ESP_LOGI(TAG, "Prepare to restart system!");
	vTaskDelay(5000 / portTICK_RATE_MS);
	esp_restart();
	return ;
}

void app_main_wifi()
{
	wifi_init_sta();
	xTaskCreate(logger_thread, "logger", 2048, NULL, 5, NULL);
	xTaskCreate(ota_thread, "ota", 4096, NULL, 5, NULL);
}
