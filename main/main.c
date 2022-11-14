#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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

#include "sdkconfig.h"

#define PORT 3333

//SPI definitions
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define TRANS_SIZE   8
#define DMA_CHAN     2

//UART definitions
#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (32) //since I don't use these I'm just setting them to a random I/O
#define ECHO_TEST_CTS (33)

#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define BUF_SIZE (1024)

static const char *TAG = "example";

char rx_buffer[BUF_SIZE];
char uart_rx_buffer[BUF_SIZE];
char tx_buffer[BUF_SIZE];
char addr_str[128];
int ip_protocol = 0;
int integ_len = 0x001F;
int integ_mult = 0x0001;
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

static void uart_rx_tx(void)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
}

static void wait_for_command(void) //currently only supports uart, will update to also wait for udp messages
{
	memset(rx_buffer, 0, sizeof(rx_buffer));
	ESP_LOGI(TAG, "Waiting for data from uart");
	while(1)
	{
		int uart_len = uart_read_bytes(ECHO_UART_PORT_NUM, uart_rx_buffer, BUF_SIZE, 20 / portTICK_RATE_MS);
		if(uart_len > 0)
		{
			char * first_cr = strchr(uart_rx_buffer, '\r');
			char * first_lf = strchr(uart_rx_buffer, '\n');
			if(first_cr != NULL)
			{
				*first_cr = '\0';
			}
			else if(first_lf != NULL)
			{
				*first_lf = '\0';
			}
			strcat(rx_buffer, uart_rx_buffer);
			if(first_lf != NULL)
			{
				strcat(rx_buffer, "\n\0");
				break;
			}
			ESP_LOGI(TAG, "rx_buffer is now %s", rx_buffer);
		}
	}
	ESP_LOGI(TAG, "command completed, exiting");
}

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
		//netflags = 2;
		//while(netflags == 2) vTaskDelay(10 / portTICK_PERIOD_MS);
		uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) tx_buffer, strlen(tx_buffer));
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
	command[1] = 0x00;
	lcd_cmd(spi, command, 1);
	command[0] = 0x37;
	command[1] = 0x00;
	lcd_cmd(spi, command, 1);
	command[0] = 0x38;
	command[1] = 0x00;
	lcd_cmd(spi, command, 1);
	command[0] = 0x39;
	command[1] = 0x00;
	lcd_cmd(spi, command, 1);
	command[0] = 0x00;
	command[1] = 0x00;
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
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //write 0x11 0x00
	command[0] = 0x82;
	command[1] = 0x00;
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
	command[1] = 0x00;
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
	command[1] = 0x00;
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
	command[1] = 0x07;
	lcd_cmd(spi, command, vis);
	command[0] = 0x49;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis);
	command[0] = 0x47;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis);
	command[0] = 0x84;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //select page 4
	command[0] = 0x51;
	command[1] = 0x01;
	lcd_cmd(spi, command, vis); //write 0x11 0x01
	command[0] = 0x00;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //flush last command
	free(command);
}

void UFSstartupcmd(spi_device_handle_t spi) //this preps the sensor to gather a single UFS frame
{
	uint8_t* command = malloc(2*sizeof(uint8_t));
	int vis = 0;
	command[0] = 0x81;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 1
	command[0] = 0x5A;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //Adjust 1
	command[0] = 0x85;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 5
	command[0] = 0x4B;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //Adjust 2
	command[0] = 0x00;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //NOP
	command[0] = 0x84;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 4
	command[0] = 0x52;
	command[1] = 0x30;
	lcd_cmd(spi, command, vis); //Set modulation selection
	command[0] = 0x55;
	command[1] = 0x2B;
	lcd_cmd(spi, command, vis); //Set UFS mode
	command[0] = 0x45;
	command[1] = 0x03;
	lcd_cmd(spi, command, vis); //set mod freq to 10MHz
	command[0] = 0x85;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 5
	//set int time 1.6384ms
	command[0] = 0x40;
	command[1] = (integ_mult >> 8) && 0xFF;
	lcd_cmd(spi, command, vis); //integration multiplier high byte
	command[0] = 0x41;
	command[1] = (integ_mult) && 0xFF;
	lcd_cmd(spi, command, vis); //integration multiplier low byte
	command[0] = 0x42;
	command[1] = (integ_len >> 8) && 0xFF;
	lcd_cmd(spi, command, vis); //integration length high byte
	command[0] = 0x43;
	command[1] = (integ_len) && 0xFF;
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

void graystartupcmd(spi_device_handle_t spi) //startup sequence for UFS mode.
{
	uint8_t* command = malloc(2*sizeof(uint8_t));
	int vis = 0;
	command[0] = 0x81;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 1
	command[0] = 0x5A;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //Adjust 1
	command[0] = 0x85;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 5
	command[0] = 0x4B;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //Adjust 2
	command[0] = 0x00;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //NOP
	command[0] = 0x84;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 4
	command[0] = 0x52;
	command[1] = 0xC0;
	lcd_cmd(spi, command, vis); //Set modulation selection
	command[0] = 0x55;
	command[1] = 0x23;
	lcd_cmd(spi, command, vis); //Set gray mode
	command[0] = 0x45;
	command[1] = 0x01;
	lcd_cmd(spi, command, vis); //set mod freq to 10MHz
	command[0] = 0x85;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //page select 5
	//set int time 1.6384ms
	command[0] = 0x40;
	command[1] = 0x00;
	lcd_cmd(spi, command, vis); //integration multiplier high byte
	command[0] = 0x41;
	command[1] = 0x01;
	lcd_cmd(spi, command, vis); //integration multiplier low byte
	command[0] = 0x42;
	command[1] = 0xFF;
	lcd_cmd(spi, command, vis); //integration length high byte
	command[0] = 0x43;
	command[1] = 0x00;
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

void getgraydata(spi_device_handle_t spi){
	uint8_t* command = malloc(2*sizeof(uint8_t));
	uint8_t* ret_data;
	command[0] = 0x82;
	command[1] = 0x00;
	ret_data = lcd_cmd(spi, command, 0); //page select 2
	for(int i = 0; i < 4; i++){
		while(ret_data[0] != 0x35 || ret_data[1] != 0x98){
			command[0] = 0x35;
			command[1] = 0x00;
			lcd_cmd(spi, command, 0); //Read STATUS
			command[0] = 0x00;
			command[1] = 0x00;
			ret_data = lcd_cmd(spi, command, 1); //NOP, read return of status read
		}
		for(int j = 0; j < 24; j++){
			command[0] = 0x2C;
			command[1] = 0x00;
			lcd_cmd(spi, command, 1);
		}
		command[0] = 0x00;
		command[1] = 0x00;
		ret_data = lcd_cmd(spi, command, 1); //NOP
	}
}

void getUFSdata(spi_device_handle_t spi, signed int* ret_dist, uint8_t* flags) //return a frame of UFS data
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
			command[1] = 0x00;
			ret_data = lcd_cmd(spi, command, 0); //NOP, read return of status read
		}
		command[0] = 0x34;
		command[1] = 0x00;
		lcd_cmd(spi, command, 0); //read MSB
		command[0] = 0x34;
		command[1] = 0x00;
		ret_data = lcd_cmd(spi, command, 0); //read LSB
		ret_dist[i] = ret_data[1] * 64; //since distance is 14 bits, its split between upper 8 and lower 6. So we only need to shift the upper bits 6 bits.
		ESP_LOGI(TAG, "Upper Bit: %x", ret_data[1]);
		command[0] = 0x00;
		command[1] = 0x00;
		ret_data = lcd_cmd(spi, command, 0); //NOP
		flags[i] = ret_data[1] & 0x03; //This byte contains saturation and overflow/underflow info in the last 2 bits. This must be cleaned out as separate flag data
		ret_dist[i] += (ret_data[1] >> 2); //only takes the last 6 bits.
		ESP_LOGI(TAG, "Lower Bit: %x", ret_data[1]);
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
	
	/* commenting out functions that use wifi temporarily, changed to using uart for debugging
	ESP_ERROR_CHECK(example_connect());
	netflags = 0;
	xTaskCreate(udp_server_startup, "udp_server", 4096, (void*)AF_INET, 5, NULL);
	*/

	//setup uart port
	uart_rx_tx();

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
		//while(netflags != 0) vTaskDelay(100 / portTICK_PERIOD_MS);
		iter = 0;
		rx_buffer[len] = 0;
		//netflags = 1;
		len = 0;
		//while(netflags != 0) vTaskDelay(100 / portTICK_PERIOD_MS);
		wait_for_command();
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
				if(rx_buffer[len] != '\n' && rx_buffer[len] != '\r'){ //ensure that no control characters get in
					inputcommand[iter] = rx_buffer[len]; //copies full command to a separate buffer then clears buffer
					iter++;
				}
				rx_buffer[len] = 0;
				len++;
			}
			inputcommand[iter] = 0;
			strcpy(tx_buffer, "executing ");
			strcat(tx_buffer, (char*)inputcommand);
			strcat(tx_buffer, "\n\0");
			//netflags = 2;
			//while(netflags == 2) vTaskDelay(100 / portTICK_PERIOD_MS);
			uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) tx_buffer, strlen(tx_buffer));
			int real_command = 0;
			if(strcmp((char*) inputcommand, (const char*) "test") == 0){
				testcmd(spi);
				real_command = 1;
			}else if(strcmp((char*) inputcommand, (const char*) "sequencer") == 0){
				sequencercmd(spi);
				real_command = 1;
			}else if(strncmp((char*) inputcommand, (const char*) "ufs data", 8) == 0){
				int iterate = 0;
				//initialize all strings here
				signed int distances[4] = {0,0,0,0};
				uint8_t flags[4] = {0,0,0,0};
				char dist_str[7] = {0,0,0,0,0,0,0};
				char flagstr[5] = {0,0,0,0,0};
				double output_dist = 0.0;
				int num = 0;
				int dem = 0;
				if(strlen((char*) inputcommand) > 8){ //if the input command has a number of iterations set after the command
					len = 8;
					while(inputcommand[len] != 0 && iterate < 1000){
						if(inputcommand[len] >= '0' && inputcommand[len] <= '9'){
							iterate = iterate * 10;
							iterate += inputcommand[len] - '0';
						}
						len++; //make sure you iterate , kids!
					}
				}
				if(iterate < 1){
					iterate = 1;
				}
				strcpy(tx_buffer, "collecting ");
				sprintf(dist_str, "%d", iterate);
				strcat(tx_buffer, dist_str);
				strcat(tx_buffer, " distance frames\n\0");
				//netflags = 2;
				//while(netflags == 2) vTaskDelay(100 / portTICK_PERIOD_MS);
				uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) tx_buffer, strlen(tx_buffer));
				for(int j = 0; j < iterate; j++){ //grab n number of data points
					UFSstartupcmd(spi);
					getUFSdata(spi, distances,flags);
					strcpy(tx_buffer, "distance: ");
					for(int k = 0; k < 4; k++){
						if(distances[k] & 0x2000){ //takes negative 14 bit numbers and converts them to negative integers.
							distances[k] = ~distances[k] + 1;
							distances[k] = distances[k] & 0x3FFF;
							distances[k] = -distances[k];
						}
						ESP_LOGI(TAG, "distance: %d", distances[k]);
					}
					num = distances[3] - distances[1];
					dem = distances[2] - distances[0];
					output_dist = 3.14159265 + atan2((double)num, (double)dem);
					output_dist = (7.49481145 * output_dist / 3.14159265); //assumes 10MHz. divide by 4 again if 40Mhz, etc etc.
					//(I don't actually know if this is supposed to be 10MHz or 2.5Mhz with 10Mhz modclk. guess we'll have to see. If its actually supposed to be 2.5Mhz then multiply by 4.)
					sprintf(dist_str, "%2.3f", output_dist);
					strcat(tx_buffer, dist_str);
					//print flags here lol
					strcat(tx_buffer, "\nflags:");
					flagstr[0] = (char) flags[0] + '0'; //repeating this 4 times is less overhead than a loop
					flagstr[1] = (char) flags[1] + '0';
					flagstr[2] = (char) flags[2] + '0';
					flagstr[3] = (char) flags[3] + '0'; //could this be bad coding practices?
					strcat(tx_buffer, flagstr);
					strcat(tx_buffer, "\n\0");
					//netflags = 2;
					//while(netflags == 2) vTaskDelay(100 / portTICK_PERIOD_MS);
					uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) tx_buffer, strlen(tx_buffer));
					vTaskDelay(100 / portTICK_PERIOD_MS);
				}
				real_command = 1;
			}else if(strcmp((char*) inputcommand, (const char*) "gray data") == 0){
				graystartupcmd(spi);
				getgraydata(spi);
				real_command = 1;
			}else if(strncmp((char*) inputcommand, (const char*) "integmul", 8) == 0){ //sets integration multiplier for sensor
				char mult_str[6] = {0,0,0,0,0,0};
				integ_mult = 0;
				unsigned char chars_read = 0; //keeps track of how many characters read
				int hex_place = 1;
				if(strlen((char*) inputcommand) > 8){ //if the input command has a number of iterations set after the command
					len = 8;
					while(inputcommand[len] != 0 && chars_read < 4){
						hex_place = 1;
						for(int i = chars_read; i < 3; i++){
							hex_place = hex_place * 16;
						}
						if(inputcommand[len] >= '0' && inputcommand[len] <= '9'){ //reads a hexadecimal number into mult
							integ_mult += hex_place * (inputcommand[len] - '0');
							mult_str[chars_read] = inputcommand[len];
							chars_read++;
						}else if(inputcommand[len] >= 'A' && inputcommand[len] <= 'F'){
							integ_mult += hex_place * (inputcommand[len] - 'A' + 10);
							mult_str[chars_read] = inputcommand[len];
							chars_read++;
						}else if(inputcommand[len] >= 'a' && inputcommand[len] <= 'f'){
							integ_mult += hex_place * (inputcommand[len] - 'a' + 10);
							mult_str[chars_read] = inputcommand[len];
							chars_read++;
						}
						ESP_LOGI(TAG, "integmul is now %x, %x", ((integ_mult >> 8) & 0xFF), (integ_mult & 0xFF));
						len++;
					}
					if(chars_read < 4){
						strcpy(tx_buffer, "ERROR: incorrect length\n\0");
					}else{
						mult_str[chars_read] = '\n';
						strcpy(tx_buffer, mult_str);
					}
					//netflags = 2;
					//while(netflags == 2) vTaskDelay(100 / portTICK_PERIOD_MS);
					uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) tx_buffer, strlen(tx_buffer));
					vTaskDelay(100 / portTICK_PERIOD_MS);
					real_command = 1;
				}
			}else if(strncmp((char*) inputcommand, (const char*) "integlen", 8) == 0){ //sets integration length for sensor
				char len_str[6] = {0,0,0,0,0,0};
				integ_len = 0;
				unsigned char chars_read = 0; //keeps track of how many characters read
				int hex_place = 1;
				if(strlen((char*) inputcommand) > 8){ //if the input command has a number of iterations set after the command
					len = 8;
					while(inputcommand[len] != 0 && chars_read < 4){
						hex_place = 1;
						for(int i = chars_read; i < 3; i++){
							hex_place = hex_place * 16;
						}
						if(inputcommand[len] >= '0' && inputcommand[len] <= '9'){ //reads a hexadecimal number into mult
							integ_len += hex_place * (inputcommand[len] - '0');
							len_str[chars_read] = inputcommand[len];
							chars_read++;
						}else if(inputcommand[len] >= 'A' && inputcommand[len] <= 'F'){
							integ_len += hex_place * (inputcommand[len] - 'A' + 10);
							len_str[chars_read] = inputcommand[len];
							chars_read++;
						}else if(inputcommand[len] >= 'a' && inputcommand[len] <= 'f'){
							integ_len += hex_place * (inputcommand[len] - 'a' + 10);
							len_str[chars_read] = inputcommand[len];
							chars_read++;
						}
						ESP_LOGI(TAG, "integlen is now %x, %x", ((integ_len >> 8) & 0xFF), (integ_len & 0xFF));
						len++;
					}
					if(chars_read < 4){
						strcpy(tx_buffer, "ERROR: incorrect length\n\0");
					}else{
						len_str[chars_read] = '\n';
						strcpy(tx_buffer, len_str);
					}
					//netflags = 2;
					//while(netflags == 2) vTaskDelay(100 / portTICK_PERIOD_MS);
					uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) tx_buffer, strlen(tx_buffer));
					vTaskDelay(100 / portTICK_PERIOD_MS);
					real_command = 1;
				}
			}
			if(real_command){
				strcpy(tx_buffer, "finished ");
				strcat(tx_buffer, (char*)inputcommand);
				strcat(tx_buffer, "\n\0");
				//netflags = 2;
				//while(netflags == 2) vTaskDelay(100 / portTICK_PERIOD_MS);
				uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) tx_buffer, strlen(tx_buffer));
			}else{
				strcpy(tx_buffer, "command not found\n\0");
				//netflags = 2;
				//while(netflags == 2) vTaskDelay(100 / portTICK_PERIOD_MS);
				uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) tx_buffer, strlen(tx_buffer));
			}

		}
	}

	free(command);
	return;
}

