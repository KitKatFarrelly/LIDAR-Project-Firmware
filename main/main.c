#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"

#include <sys/param.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define PORT 3333

//SPI definitions
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define TRANS_SIZE   8
#define DMA_CHAN     2

static const char *TAG = "example";

char rx_buffer[128];
char tx_buffer[128];
char addr_str[128];
int ip_protocol = 0;
struct sockaddr_in6 dest_addr;

struct sockaddr_storage source_addr;
socklen_t socklen = sizeof(source_addr);

uint8_t netflags = 0;

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */

int udp_rcmsg(int sock)
{
	ESP_LOGI(TAG, "Waiting for data");
	int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
	// Error occurred during receiving
	if (len < 0) {
		ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
		return len;
	}
	netflags = 0;
	return len;
}

int udp_sndmsg(int sock)
{
	// Get the sender's ip address as string
	ESP_LOGI(TAG, "Sending Data");
	if (source_addr.ss_family == PF_INET) {
		inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
	} else if (source_addr.ss_family == PF_INET6) {
		inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
	}
	int err = sendto(sock, tx_buffer, strlen(tx_buffer), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
	netflags = 0;
	if (err < 0) {
		ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
		return err;
	}

	return err;
}

uint8_t* lcd_cmd(spi_device_handle_t spi, uint8_t* cmd, uint8_t outputflag)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 16;
	t.rxlength = 16;
	t.flags |= SPI_TRANS_USE_RXDATA;
	t.tx_buffer = cmd;
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
    uint8_t* output = t.rx_data;
    if(outputflag){
    	uint8_t i = 0;
		char inputhex[5] = {0,0,0,0,0};
		uint8_t* cmdptr = cmd;
		while(i < 2){
				sprintf(inputhex+i*2,"%02x", *cmdptr);
				cmdptr++;
				i++;
		}
		char hex[5] = {0,0,0,0,0};
		i = 0;
		while(i < 2){
				sprintf(hex+i*2,"%02x", *output);
				output++;
				i++;
		}
		strcpy(tx_buffer, inputhex);
		strcat(tx_buffer, "          ");
		strcat(tx_buffer, hex);
		strcat(tx_buffer, "\n\0");
		netflags = 2;
		while(netflags == 2) vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    cmd[0] = t.rx_data[0];
    cmd[1] = t.rx_data[1];
    return cmd;
}

void testcmd(spi_device_handle_t spi) //test command that returns the chip and wafer id of the lidar
{
	uint8_t* command = malloc(2*sizeof(uint8_t));
	command[0] = 0x87;
	command[1] = 0x00;
	lcd_cmd(spi, command, 1);
	command[0] = 0x36;
	lcd_cmd(spi, command, 1);
	command[0] = 0x37;
	lcd_cmd(spi, command, 1);
	command[0] = 0x38;
	lcd_cmd(spi, command, 1);
	command[0] = 0x39;
	lcd_cmd(spi, command, 1);
	command[0] = 0x00;
	lcd_cmd(spi, command, 1);
	free(command);
}

void sequencercmd(spi_device_handle_t spi) //startup sequence for lidar
{
	uint8_t* command = malloc(2*sizeof(uint8_t));
	int vis = 0;
	command[0] = 0x84;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 4
	command[0] = 0x51;
	lcd_cmd(spi, command, vis); //write 0x11 0x00
	command[0] = 0x82;
	lcd_cmd(spi, command, vis); //page select 2
	command[0] = 0x47;
	command[1] = 0x01;
	lcd_cmd(spi, command, vis); //write 0x07 (SR_Program) 0x01
	command[0] = 0x40;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //write 0x00 (SR_Address) 0x00
	command[0] = 0x41;
	command[1] = 0x43;
	lcd_cmd(spi, command, vis); //write 0x01 (SR_Data_0) 0x43
	command[0] = 0x42;
	command[1] = 0x18;
	lcd_cmd(spi, command, vis); //write 0x02 (SR_Data_1) 0x18
	command[0] = 0x43;
	command[1] = 0x02;
	lcd_cmd(spi, command, vis);
	command[0] = 0x44;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis);
	command[0] = 0x45;
	lcd_cmd(spi, command, vis);
	command[0] = 0x46;
	command[1] = 0x2D;
	lcd_cmd(spi, command, vis);
	command[0] = 0x47;
	command[1] = 0x07;
	lcd_cmd(spi, command, vis);
	command[0] = 0x40;
	command[1] = 0x01;
	lcd_cmd(spi, command, vis);
	command[0] = 0x41;
	command[1] = 0x43;
	lcd_cmd(spi, command, vis);
	command[0] = 0x42;
	command[1] = 0x18;
	lcd_cmd(spi, command, vis);
	command[0] = 0x43;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis);
	command[0] = 0x44;
	lcd_cmd(spi, command, vis);
	command[0] = 0x45;
	command[1] = 0xA8;
	lcd_cmd(spi, command, vis);
	command[0] = 0x46;
	command[1] = 0x2E;
	lcd_cmd(spi, command, vis);
	command[0] = 0x47;
	command[1] = 0x07;
	lcd_cmd(spi, command, vis);
	command[0] = 0x40;
	command[1] = 0x02;
	lcd_cmd(spi, command, vis);
	command[0] = 0x41;
	command[1] = 0x43;
	lcd_cmd(spi, command, vis);
	command[0] = 0x42;
	command[1] = 0x18;
	lcd_cmd(spi, command, vis);
	command[0] = 0x43;
	command[1] = 0x11;
	lcd_cmd(spi, command, vis);
	command[0] = 0x44;
	command[1] = 0x03;
	lcd_cmd(spi, command, vis);
	command[0] = 0x45;
	command[1] = 0x50;
	lcd_cmd(spi, command, vis);
	command[0] = 0x46;
	command[1] = 0x2F;
	lcd_cmd(spi, command, vis);
	command[0] = 0x47;
	command[1] = 0x07;
	lcd_cmd(spi, command, vis);
	command[0] = 0x48;
	lcd_cmd(spi, command, vis);
	command[0] = 0x49;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis);
	command[0] = 0x47;
	lcd_cmd(spi, command, vis);
	command[0] = 0x84;
	lcd_cmd(spi, command, vis); //select page 4
	command[0] = 0x51;
	command[1] = 0x01;
	lcd_cmd(spi, command, vis); //write 0x11 0x01
	command[0] = 0x00;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //flush last command
	free(command);
}

void UFSstartupcmd(spi_device_handle_t spi) //startup sequence for UFS mode.
{
	uint8_t* command = malloc(2*sizeof(uint8_t));
	int vis = 0;
	command[0] = 0x81;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 1
	command[0] = 0x5A;
	lcd_cmd(spi, command, vis); //Adjust 1
	command[0] = 0x85;
	lcd_cmd(spi, command, vis); //page select 5
	command[0] = 0x4B;
	lcd_cmd(spi, command, vis); //Adjust 2
	command[0] = 0x00;
	lcd_cmd(spi, command, vis); //NOP
	command[0] = 0x84;
	lcd_cmd(spi, command, vis); //page select 4
	command[0] = 0x52;
	command[1] = 0x30;
	lcd_cmd(spi, command, vis); //Set modulation selection
	command[0] = 0x55;
	command[1] = 0x2B;
	lcd_cmd(spi, command, vis); //Set UFS mode
	command[0] = 0x45;
	command[1] = 0x01;
	lcd_cmd(spi, command, vis); //set mod freq to 10MHz
	command[0] = 0x85;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 5
	//set int time 1.6384ms
	command[0] = 0x40;
	lcd_cmd(spi, command, vis); //integration multiplier high byte
	command[0] = 0x41;
	command[1] = 0x01;
	lcd_cmd(spi, command, vis); //integration multiplier low byte
	command[0] = 0x42;
	command[1] = 0xFF;
	lcd_cmd(spi, command, vis); //integration length high byte
	command[0] = 0x43;
	lcd_cmd(spi, command, vis); //integration length low byte
	command[0] = 0x82;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 2
	command[0] = 0x58;
	command[1] = 0x01;
	lcd_cmd(spi, command, vis); //start measurement
	command[0] = 0x00;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //NOP
	free(command);
}

void getUFSdata(spi_device_handle_t spi, unsigned int* ret_dist) //return a frame of UFS data
{
	uint8_t* command = malloc(2*sizeof(uint8_t));
	uint8_t* ret_data;
	command[0] = 0x82;
	command[1] = 0x00;
	ret_data = lcd_cmd(spi, command, 0); //page select 2
	for(int i = 0; i < 4; i++){
		while(ret_data[0] != 0x35 || ret_data[1] != 0x82){
			command[0] = 0x35;
			command[1] = 0x00;
			lcd_cmd(spi, command, 0); //Read STATUS
			command[0] = 0x00;
			ret_data = lcd_cmd(spi, command, 1); //NOP, read return of status read
		}
		command[0] = 0x34;
		lcd_cmd(spi, command, 0); //read MSB
		command[0] = 0x34;
		ret_data = lcd_cmd(spi, command, 1); //read LSB
		ret_dist[i] = ret_data[1] * 256;
		command[0] = 0x00;
		ret_data = lcd_cmd(spi, command, 1); //NOP
		ret_dist[i] += ret_data[1];
	}
	free(command);
}

void udp_server_startup(void *pvParameters)
{
	int addr_family = (int)pvParameters;

	while (1){
		if (addr_family == AF_INET) {
			struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
			dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
			dest_addr_ip4->sin_family = AF_INET;
			dest_addr_ip4->sin_port = htons(PORT);
			ip_protocol = IPPROTO_IP;
		} else if (addr_family == AF_INET6) {
			bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
			dest_addr.sin6_family = AF_INET6;
			dest_addr.sin6_port = htons(PORT);
			ip_protocol = IPPROTO_IPV6;
		}

		int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
		if (sock < 0) {
			ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
			break;
		}
		ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
		if (addr_family == AF_INET6) {
			// Note that by default IPV6 binds to both protocols, it is must be disabled
			// if both protocols used at the same time (used in CI)
			int opt = 1;
			setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
			setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
		}
#endif

		int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
		if (err < 0) {
			ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
		}
		ESP_LOGI(TAG, "Socket bound, port %d", PORT);
		while (1){
			int len = 0;
			if(netflags == 1){
				len = udp_rcmsg(sock);
			}else if(netflags == 2){
				len = udp_sndmsg(sock);
			}
			if(len < 0){
				break;
			}
			vTaskDelay(100 / portTICK_PERIOD_MS); //check 10 times per second
		}
		if (sock != -1) {
			ESP_LOGE(TAG, "Shutting down socket and restarting...");
			shutdown(sock, 0);
			close(sock);
		}

	}
	vTaskDelete(NULL);
}

void app_main(void)
{
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	/* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
	 * Read "Establishing Wi-Fi or Ethernet Connection" section in
	 * examples/protocols/README.md for more information about this function.
	 */
	ESP_ERROR_CHECK(example_connect());
	netflags = 0;
	xTaskCreate(udp_server_startup, "udp_server", 4096, (void*)AF_INET, 5, NULL);

	esp_err_t ret;
	spi_device_handle_t spi;
	spi_bus_config_t buscfg={
		.miso_io_num=PIN_NUM_MISO,
		.mosi_io_num=PIN_NUM_MOSI,
		.sclk_io_num=PIN_NUM_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=TRANS_SIZE
	};
	spi_device_interface_config_t devcfg={
		.clock_speed_hz=4*1000*1000,           //Clock out at 4 MHz
		.mode=0,                                //SPI mode 0
		.spics_io_num=PIN_NUM_CS,               //CS pin
		.queue_size=7,                          //We want to be able to queue 7 transactions at a time
	};
	//Initialize the SPI bus
	ret=spi_bus_initialize(SPI3_HOST, &buscfg, DMA_CHAN);
	ESP_ERROR_CHECK(ret);
	//Attach the LCD to the SPI bus
	ret=spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
	uint8_t* command = malloc(2*sizeof(uint8_t));
	uint8_t inputbyte[4] = {0,0,0,0}; //for when the user inputs a 2 byte sequence to be sent to the device
	uint8_t inputcommand[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //for when the user inputs a word command to execute on the device
	uint8_t iter = 0;
	int len = 0;
	command[0] = 0;
	command[1] = 0;
	lcd_cmd(spi, command, 0);
	lcd_cmd(spi, command, 0);
	lcd_cmd(spi, command, 0);
	while(1){
		while(netflags != 0) vTaskDelay(100 / portTICK_PERIOD_MS);
		iter = 0;
		rx_buffer[len] = 0;
		netflags = 1;
		len = 0;
		while(netflags != 0) vTaskDelay(100 / portTICK_PERIOD_MS);
		if(rx_buffer[len] >= '0' && rx_buffer[len] <= '9'){
			while(rx_buffer[len] != 0 && iter < 4){
				if(rx_buffer[len] >= '0' && rx_buffer[len] <= '9'){
					inputbyte[iter] = rx_buffer[len] - '0';
					iter++;
				}
				rx_buffer[len] = 0;
				len++;
			}
			if(rx_buffer[0] == 0x13 || rx_buffer[0] == 0x10){
				return;
			}
			command[0] = inputbyte[0] * 16 + inputbyte[1];
			command[1] = inputbyte[2] * 16 + inputbyte[3];
			lcd_cmd(spi, command, 1);
		}else{
			while(rx_buffer[len] != 0 && iter < 16){
				if((rx_buffer[len] >= 'a' && rx_buffer[len] <= 'z') || (rx_buffer[len] == ' ')){ //only accepts lower case letters and space
					inputcommand[iter] = rx_buffer[len];
					iter++;
				}
				rx_buffer[len] = 0;
				len++;
			}
			inputcommand[iter] = 0;
			strcpy(tx_buffer, "executing ");
			strcat(tx_buffer, (char*)inputcommand);
			strcat(tx_buffer, "\n\0");
			netflags = 2;
			while(netflags == 2) vTaskDelay(100 / portTICK_PERIOD_MS);
			int real_command = 0;
			unsigned int distances[64] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
			if(strcmp((char*) inputcommand, (const char*) "test") == 0){
				testcmd(spi);
				real_command = 1;
			}else if(strcmp((char*) inputcommand, (const char*) "sequencer") == 0){
				sequencercmd(spi);
				real_command = 1;
			}else if(strcmp((char*) inputcommand, (const char*) "ufs start") == 0){
				UFSstartupcmd(spi);
				real_command = 1;
			}else if(strcmp((char*) inputcommand, (const char*) "ufs data") == 0){
				getUFSdata(spi, distances);
				char dist_str[6] = {0,0,0,0,0,0};
				strcpy(tx_buffer, "distances: ");
				for(int dist_iter = 0; dist_iter < 4; dist_iter++){
					sprintf(dist_str,"%05u", distances[dist_iter]);
					strcat(tx_buffer, dist_str);
					strcat(tx_buffer, ", ");
				}

				strcat(tx_buffer, "\n\0");
				netflags = 2;
				while(netflags == 2) vTaskDelay(100 / portTICK_PERIOD_MS);
				real_command = 1;
			}
			if(real_command){
				strcpy(tx_buffer, "finished ");
				strcat(tx_buffer, (char*)inputcommand);
				strcat(tx_buffer, "\n\0");
				netflags = 2;
				while(netflags == 2) vTaskDelay(100 / portTICK_PERIOD_MS);
			}else{
				strcpy(tx_buffer, "command not found\n\0");
				netflags = 2;
				while(netflags == 2) vTaskDelay(100 / portTICK_PERIOD_MS);
			}

		}
	}

	free(command);
	return;
}
