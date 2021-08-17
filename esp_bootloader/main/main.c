#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
//#include "soc/uart_struct.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
//#include "app_wifi.h"
//#include "esp_tls.h"

#include <stdio.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdmmc_common.h"
//#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_attr.h"
#include <sys/time.h>
#include <unistd.h>
//#include "lwip/err.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "sdkconfig.h"

#include "esp_peripherals.h"
#include "board.h"
#include "esp_ota_ops.h"

#include "nvs.h"
#include "nvs_flash.h"

#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>
// #include "libssh2_config.h"
// #include <libssh2.h>
// #include <libssh2_sftp.h>
#define MAX_HTTP_RECV_BUFFER 512
static const char *TAG = "BOOT_LOG";
static const char *AT = "AT_COMMAND";
#define BUFFSIZE 1024
char upload_filename[1025] = "";
static char ota_write_data[BUFFSIZE + 1] = {0};
int BP = 0;
#define PRODUCTION "/sdcard/esp.bin"
#define BACKUP_PRODUCTION "/sdcard/backup/esp.bin"
#define USE_SPI_MODE

#define SPI_DMA_CHAN 1

#ifdef USE_SPI_MODE

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5
#endif //USE_SPI_MODE

long code_size = 0;
//   FILE *local;

static const int RX_BUF_SIZE = 1024 * 2;
static QueueHandle_t uart2_queue;
//YMODEM INCLUDES AND DEFINITIONS
// === UART DEFINES ====
#define EX_UART_NUM UART_NUM_2
#define BUF_SIZE (4096)

static void uart_event_task(void *pvParameters)
{
  uart_event_t event;
  for (;;)
  {
    //Waiting for UART event.
    if (xQueueReceive(uart2_queue, (void *)&event, (portTickType)portMAX_DELAY))
    {
      //ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
      switch (event.type)
      {
      //Event of UART receving data
      case UART_DATA:
        break;
      //Event of HW FIFO overflow detected
      case UART_FIFO_OVF:
        ESP_LOGI(TAG, "hw fifo overflow\n");
        uart_flush(EX_UART_NUM);
        break;
      //Event of UART ring buffer full
      case UART_BUFFER_FULL:
        ESP_LOGI(TAG, "ring buffer full\n");
        uart_flush(EX_UART_NUM);
        break;
      //Event of UART RX break detected
      case UART_BREAK:
        ESP_LOGI(TAG, "uart rx break\n");
        break;
      //Event of UART parity check error
      case UART_PARITY_ERR:
        ESP_LOGI(TAG, "uart parity error\n");
        break;
      //Event of UART frame error
      case UART_FRAME_ERR:
        ESP_LOGI(TAG, "uart frame error\n");
        break;
      //UART_PATTERN_DET
      case UART_PATTERN_DET:
        ESP_LOGI(TAG, "uart pattern detected\n");
        break;
      //Others
      default:
        ESP_LOGI(TAG, "uart event type: %d\n", event.type);
        break;
      }
    }
  }
  vTaskDelete(NULL);
}
static void __attribute__((noreturn)) task_fatal_error()
{
  ESP_LOGE(TAG, "Exiting task due to fatal error...");

  esp_err_t err;

  err = esp_ota_mark_app_invalid_rollback_and_reboot();

  if (err == ESP_FAIL)
  {
    printf("NOT SUCCESSFUL");
  }
  if (err == ESP_ERR_OTA_ROLLBACK_FAILED)
  {
    printf("The rollback is not possible due to flash does not have any apps.");
  }

  (void)vTaskDelete(NULL);

  // while (1)
  // {
  //   ;
  // }

  //ROLLBACK TO PREVIOUSLY WORKABLE APP
}
static void ota_example_task(void *pvParameter)
{
  if (BP == 0)
  {
    FILE *f_up = fopen(PRODUCTION, "r");
    if (f_up == NULL)
    {
      ESP_LOGE(TAG, "Failed to open esp.bin file for reading");
      return 0;
    }
    fseek(f_up, 0, SEEK_END);
    code_size = ftell(f_up);
    fclose(f_up);

    printf("ESP.bin size : %ld\n", code_size);
  }

  else
  {
    FILE *f_up = fopen(BACKUP_PRODUCTION, "r");
    if (f_up == NULL)
    {
      ESP_LOGE(TAG, "Failed to open esp.bin file for reading");
      return 0;
    }
    fseek(f_up, 0, SEEK_END);
    code_size = ftell(f_up);
    fclose(f_up);

    printf("BACKUP ESP.bin size : %ld\n", code_size);
  }

  esp_err_t err1 = nvs_flash_init();
  if (err1 == ESP_ERR_NVS_NO_FREE_PAGES)
  {
    // OTA app partition table has a smaller NVS partition size than the non-OTA
    // partition table. This size mismatch may cause NVS initialization to fail.
    // If this happens, we erase NVS partition and initialize NVS again.
    ESP_ERROR_CHECK(nvs_flash_erase());
    err1 = nvs_flash_init();
  }

  esp_err_t err;
  /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
  esp_ota_handle_t update_handle = 0;
  const esp_partition_t *update_partition = NULL;

  ESP_LOGI(TAG, "FLASHING OF ESP.BIN WILL BEGIN IN FEW SECONDS");

  const esp_partition_t *configured = esp_ota_get_boot_partition();
  const esp_partition_t *running = esp_ota_get_running_partition();

  if (configured != running)
  {
    ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
             configured->address, running->address);
    ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
  }
  ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
           running->type, running->subtype, running->address);

  update_partition = esp_ota_get_next_update_partition(NULL);
  ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
           update_partition->subtype, update_partition->address);
  assert(update_partition != NULL);

  err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "esp_ota_begin failed, error=%d", err);
    task_fatal_error();
  }
  ESP_LOGI(TAG, "esp_ota_begin succeeded");

  memset(ota_write_data, 0, BUFFSIZE);

  if (BP == 0)
  {
    FILE *f_boot = fopen(PRODUCTION, "r");

    int buff_len = 1024;
    printf("Number : %f", ceil(code_size / 1024));
    for (uint32_t i = 0; i < ceil(code_size / 1024) + 1; i++)
    {

      fread(ota_write_data, BUFFSIZE, 1, f_boot);

      err = esp_ota_write(update_handle, (const void *)ota_write_data, buff_len);

      memset(ota_write_data, 0, sizeof(ota_write_data));

      if (err != ESP_OK)
      {
        ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
        task_fatal_error();
      }
      ESP_LOGI(TAG, "Have written image length %d percent", (i * buff_len));
    }
    //ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);

    if (esp_ota_end(update_handle) != ESP_OK)
    {
      ESP_LOGE(TAG, "esp_ota_end failed!");
      task_fatal_error();
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
      ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
      task_fatal_error();
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
  }
  else
  {
    FILE *f_boot = fopen(BACKUP_PRODUCTION, "r");

    int buff_len = 1024;
    printf("Number : %f", ceil(code_size / 1024));
    for (uint32_t i = 0; i < ceil(code_size / 1024) + 1; i++)
    {

      fread(ota_write_data, BUFFSIZE, 1, f_boot);

      err = esp_ota_write(update_handle, (const void *)ota_write_data, buff_len);

      memset(ota_write_data, 0, sizeof(ota_write_data));

      if (err != ESP_OK)
      {
        ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
        task_fatal_error();
      }
      ESP_LOGI(TAG, "Have written image length %d percent", (i * buff_len));
    }
    //ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);

    if (esp_ota_end(update_handle) != ESP_OK)
    {
      ESP_LOGE(TAG, "esp_ota_end failed!");
      task_fatal_error();
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
      ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
      task_fatal_error();
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
  }
}
void rx_task_2()
{
  static const char *RX_TASK_TAG = "RX_TASK";
  esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
  uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
  char time[7];
  char lat[11];
  char lon[11];
  char u_id[25];
  while (1)
  {
    const int rxBytes = uart_read_bytes(UART_NUM_2, data, 70, 1000 / portTICK_RATE_MS);
    if (rxBytes > 0)
    {
      uart_flush(EX_UART_NUM);
      data[rxBytes] = 0;
      ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
      ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

      /* STM AT COMMANDS HEADER IS $ */
      /* REFERENCE SHEET FOR AT COMMANDS:-https://drive.google.com/file/d/1bp3Ip4MONyYNNdNoeufedAAPzffX1jIN/view?usp=sharing */
      if (data[0] == '$')
      {
        if ((data[1] == 'P') && (data[2] == 'C'))
        {
          BP = 0;
          xTaskCreate(&ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);
        }
        else if ((data[1] == 'B') && (data[2] == 'P'))
        {
          BP = 1;
          xTaskCreate(&ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);
        }
      }
    }
  }
}

void app_main(void)
{
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
  };
  //Set UART parameters
  uart_param_config(EX_UART_NUM, &uart_config);
  //Set UART log level
  esp_log_level_set(TAG, ESP_LOG_INFO);
  //Install UART driver, and get the queue.
  uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart2_queue, 2);

  //Set UART pins (using UART0 default pins, ie no changes.)
  uart_set_pin(EX_UART_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  ESP_LOGI(TAG, "Initializing SD card");

#ifndef USE_SPI_MODE
  ESP_LOGI(TAG, "Using SDMMC peripheral");
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

  // To use 1-line SD mode, uncomment the following line:
  // slot_config.width = 1;

  // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
  // Internal pull-ups are not sufficient. However, enabling internal pull-ups
  // does make a difference some boards, so we do that here.
  gpio_set_pull_mode(15, GPIO_PULLUP_ONLY); // CMD, needed in 4- and 1- line modes
  gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);  // D0, needed in 4- and 1-line modes
  gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);  // D1, needed in 4-line mode only
  gpio_set_pull_mode(12, GPIO_PULLUP_ONLY); // D2, needed in 4-line mode only
  gpio_set_pull_mode(13, GPIO_PULLUP_ONLY); // D3, needed in 4- and 1-line modes

#else
  ESP_LOGI(TAG, "Using SPI peripheral");

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
  slot_config.gpio_miso = PIN_NUM_MISO;
  slot_config.gpio_mosi = PIN_NUM_MOSI;
  slot_config.gpio_sck = PIN_NUM_CLK;
  slot_config.gpio_cs = PIN_NUM_CS;
  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
#endif //USE_SPI_MODE

  // Options for mounting the filesystem.
  // If format_if_mount_failed is set to true, SD card will be partitioned and
  // formatted in case when mounting fails.
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files = 3,
      .allocation_unit_size = 16 * 1024};

  // Use settings defined above to initialize SD card and mount FAT filesystem.
  // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
  // Please check its source code and implement error recovery when developing
  // production applications.
  sdmmc_card_t *card;
  esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret != ESP_OK)
  {
    if (ret == ESP_FAIL)
    {
      ESP_LOGE(TAG, "Failed to mount filesystem. "
                    "If you want the card to be formatted, set format_if_mount_failed = true.");
    }
    else
    {
      ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                    "Make sure SD card lines have pull-up resistors in place.",
               esp_err_to_name(ret));
    }
    return;
  }

  // Card has been initialized, print its properties
  sdmmc_card_print_info(stdout, card);
  sdmmc_init_cid(card);
  printf("CID: %d\n", card->cid.mfg_id);
  printf("OEM: %d\n", card->cid.oem_id);
  printf("NAME: %s\n", card->cid.name);
  // Use POSIX and C standard library functions to work with files.

  /*DEVICE IS IN BOOTLOADER CODE*/
  uart_write_bytes(EX_UART_NUM, "$BT", 4);
  //uart_write_bytes(EX_UART_NUM, "@", 2);
  printf("BOOTLOADER STARTED \n");

  while (1)
  {
    rx_task_2();
  }
}
