#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/uart_struct.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "app_wifi.h"
#include "esp_tls.h"
// #include "ff.h"

#include "esp_http_client.h"
//#include "espcurl.h"

#include "libssh2_config.h"
#include <libssh2.h>
#include <libssh2_sftp.h>

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
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_attr.h"
#include <sys/time.h>
#include <unistd.h>
#include "lwip/err.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "fatfs_stream.h"
#include "i2s_stream.h"
#include "mp3_decoder.h"

#include "esp_peripherals.h"
#include "board.h"
#include "esp_ota_ops.h"
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>
const esp_partition_t *factory;
esp_partition_iterator_t part_iter;
esp_err_t err;
#define MAX_HTTP_RECV_BUFFER 512
static const char *TAG = "HTTP_CLIENT";
static const char *AT = "AT_COMMAND";
#define VERSION_NO "2.0"
#define MOUNT_POINT "/sdcard"

#define USE_SPI_MODE

#define SPI_DMA_CHAN 1

#ifdef USE_SPI_MODE

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 5
#endif //USE_SPI_MODE

int wifi_already_on = 0;
long file_size_ift = 0;

//audio filename
char *gear_up = "/sdcard/gearup.mp3";
char *gear_down = "/sdcard/geardown.mp3";
char *welcome = "/sdcard/welcome.mp3";
char *hotspoton = "/sdcard/hson.mp3";
char *hotspotreq = "/sdcard/hsreq.mp3";
char *hotspotoff = "/sdcard/hspoff.mp3";

char *keep_ign_on = "/sdcard/keep.mp3";
char *update_available = "/sdcard/update.mp3";
char *press_button = "/sdcard/button.mp3";
char *turn_vehicle_on = "/sdcard/veh_on.mp3";
char *turn_vehicle_off = "/sdcard/veh_off.mp3";
char *flashing_complete = "/sdcard/complete.mp3";
char *extra = "/sdcard/extra.mp3";
char *backup_audio = "/sdcard/backup.mp3";

char *PRESS_CLUTCH = "/sdcard/pclutch.mp3";
char *RELEASE_CLUTCH = "/sdcard/rclutch.mp3";
char *PRESS_BRAKE = "/sdcard/pbrake.mp3";
char *RELEASE_BRAKE = "/sdcard/rbrake.mp3";
char *FIRST_GEAR = "/sdcard/fgear.mp3";
char *NEUTRAL = "/sdcard/neutral.mp3";
char *ENG_ON = "/sdcard/eng_on.mp3";
char *PRESS_THROTTLE = "/sdcard/pthrott.mp3";
char *RELEASE_THROTTLE = "/sdcard/rthrott.mp3";
char *ENG_OFF = "/sdcard/eng_off.mp3";
char *SERVER_CONNECT_SUCCESS = "/sdcard/serverp.mp3";
char *BRD_DONE = "/sdcard/brddone.mp3";
char *REQ_DONE = "/sdcard/reqdone.mp3";

//sftp stuff
unsigned long hostaddr = "65.0.250.171";
int sock, i, auth_pw = 1;
struct sockaddr_in sin;
const char *fingerprint;
LIBSSH2_SESSION *session;
const char *username = "hipersftp";
const char *password = "Hip3rsFtP2020";
const char *loclfile = "sftp_write.c";
int rc;
//   FILE *local;
LIBSSH2_SFTP *sftp_session;
LIBSSH2_SFTP_HANDLE *sftp_handle;
char mem[1024];
size_t nread;
char *ptr;

char *SESH_RET = "Session Expired\n";

char filename_buffer[20];
char filename_buffer_2[40];

char *private_id;
char public_id[20];
char *session_id;
char *upload_size;

typedef enum HTTP_RESPONSE_ID
{
  PRIVATE_ID,
  SESSION_ID,
  LAST_UPLOAD,
  FILE_UPLOAD
} http_resp;

http_resp http_response_id;

//flaags...////F
int got_sessionid_flag = 0;
int file_upload_flag = 0;
int another_sesh_flag = 0;

const char CONTENT_HEADERS_1[] =
    "\r\nContent-Disposition: form-data; name=\"testfile\"; filename=\"";

const char CONTENT_HEADERS_2[] =
    "\"\r\n"
    "Content-Type: application/octet\r\n\r\n";

char body[4096];
char boundary[32] = "----";

static const int RX_BUF_SIZE = 1024 * 2;

//YMODEM INCLUDES AND DEFINITIONS
// === UART DEFINES ====
#define EX_UART_NUM UART_NUM_2
#define BUF_SIZE (4096)

// === LED pin used to show transfer activity ===
// === Set to 0 if you don't want to use it   ===
#define YMODEM_LED_ACT 0
#define YMODEM_LED_ACT_ON 1 // pin level for LED ON

// ==== Y-MODEM defines ====
#define PACKET_SEQNO_INDEX (1)
#define PACKET_SEQNO_COMP_INDEX (2)

#define PACKET_HEADER (3)
#define PACKET_TRAILER (2)
#define PACKET_OVERHEAD (PACKET_HEADER + PACKET_TRAILER)
#define PACKET_SIZE (128)
#define PACKET_1K_SIZE (1024)

#define FILE_SIZE_LENGTH (16)

#define SOH (0x01)   /* start of 128-byte data packet */
#define STX (0x02)   /* start of 1024-byte data packet */
#define EOT (0x04)   /* end of transmission */
#define ACK (0x06)   /* acknowledge */
#define NAK (0x15)   /* negative acknowledge */
#define CA (0x18)    /* two of these in succession aborts transfer */
#define CRC16 (0x43) /* 'C' == 0x43, request 16-bit CRC */

#define ABORT1 (0x41) /* 'A' == 0x41, abort by user */
#define ABORT2 (0x61) /* 'a' == 0x61, abort by user */

#define NAK_TIMEOUT (1000)
#define MAX_ERRORS (20)

#define YM_MAX_FILESIZE (10 * 1024 * 1024)

//THIS WILL LOAD THE FACTORY CODE AGAIN (BOOTLOADER)
void reboot_esp()
{
  part_iter = esp_partition_find(ESP_PARTITION_TYPE_APP,            // Get partition iterator for
                                 ESP_PARTITION_SUBTYPE_APP_FACTORY, // factory partition
                                 "factory");
  if (part_iter == NULL) // Check result
  {
    uart_write_bytes(EX_UART_NUM, "$C404", 6);
    ESP_LOGE(TAG, "Failed to find factory partition");
  }
  else
  {
    factory = esp_partition_get(part_iter);    // Get partition struct
    esp_partition_iterator_release(part_iter); // Release the iterator
    err = esp_ota_set_boot_partition(factory); // Set partition for boot
    if (err != ESP_OK)                         // Check error
    {
      uart_write_bytes(EX_UART_NUM, "$C404", 6);
      ESP_LOGE(TAG, "Failed to set boot partition");
    }
    else
    {
      uart_write_bytes(EX_UART_NUM, "$C100", 6);
      esp_restart(); // Restart ESP and go to factory partition(bootloader code)
    }
  }
}

void audio_playback_handler(char *filename)
{
  audio_pipeline_handle_t pipeline;
  audio_element_handle_t fatfs_stream_reader, i2s_stream_writer, mp3_decoder;

  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set(TAG, ESP_LOG_INFO);

  //ESP_LOGI(TAG, "[ 1 ] Mount sdcard");
  // Initialize peripherals management
  esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
  esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

  /*ESP_LOGI(TAG, "[ 2 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START); */

  ESP_LOGI(TAG, "[3.0] Create audio pipeline for playback");
  audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
  pipeline = audio_pipeline_init(&pipeline_cfg);
  mem_assert(pipeline);

  ESP_LOGI(TAG, "[3.1] Create fatfs stream to read data from sdcard");
  fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
  fatfs_cfg.type = AUDIO_STREAM_READER;
  fatfs_stream_reader = fatfs_stream_init(&fatfs_cfg);

  ESP_LOGI(TAG, "[3.2] Create i2s stream to write data to codec chip");
  i2s_stream_cfg_t i2s_cfg = I2S_STREAM_INTERNAL_DAC_CFG_DEFAULT();
  i2s_cfg.type = AUDIO_STREAM_WRITER;
  i2s_stream_writer = i2s_stream_init(&i2s_cfg);

  ESP_LOGI(TAG, "[3.3] Create mp3 decoder to decode mp3 file");
  mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
  mp3_decoder = mp3_decoder_init(&mp3_cfg);

  ESP_LOGI(TAG, "[3.4] Register all elements to audio pipeline");
  audio_pipeline_register(pipeline, fatfs_stream_reader, "file");
  audio_pipeline_register(pipeline, mp3_decoder, "mp3");
  audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

  ESP_LOGI(TAG, "[3.5] Link it together [sdcard]-->fatfs_stream-->mp3_decoder-->i2s_stream-->[codec_chip]");
  audio_pipeline_link(pipeline, (const char *[]){"file", "mp3", "i2s"}, 3);

  ESP_LOGI(TAG, "[3.6] Set up  uri (file as fatfs_stream, mp3 as mp3 decoder, and default output is i2s)");
  audio_element_set_uri(fatfs_stream_reader, filename);

  ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
  audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
  audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

  ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
  audio_pipeline_set_listener(pipeline, evt);

  ESP_LOGI(TAG, "[4.2] Listening event from peripherals");
  audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

  ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
  audio_pipeline_run(pipeline);

  ESP_LOGI(TAG, "[ 6 ] Listen for all pipeline events");
  while (1)
  {
    audio_event_iface_msg_t msg;
    esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
    if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
      continue;
    }

    if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)mp3_decoder && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO)
    {
      audio_element_info_t music_info = {0};
      audio_element_getinfo(mp3_decoder, &music_info);

      ESP_LOGI(TAG, "[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
               music_info.sample_rates, music_info.bits, music_info.channels);

      audio_element_setinfo(i2s_stream_writer, &music_info);
      i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
      continue;
    }

    /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
    if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)i2s_stream_writer && msg.cmd == AEL_MSG_CMD_REPORT_STATUS && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED)))
    {
      ESP_LOGW(TAG, "[ * ] Stop event received");
      break;
    }
  }

  ESP_LOGI(TAG, "[ 7 ] Stop audio_pipeline");
  audio_pipeline_terminate(pipeline);

  audio_pipeline_unregister(pipeline, fatfs_stream_reader);
  audio_pipeline_unregister(pipeline, i2s_stream_writer);
  audio_pipeline_unregister(pipeline, mp3_decoder);

  /* Terminal the pipeline before removing the listener */
  audio_pipeline_remove_listener(pipeline);

  /* Stop all periph before removing the listener */
  esp_periph_set_stop_all(set);
  audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

  /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
  audio_event_iface_destroy(evt);

  /* Release all resources */
  audio_pipeline_deinit(pipeline);
  audio_element_deinit(fatfs_stream_reader);
  audio_element_deinit(i2s_stream_writer);
  audio_element_deinit(mp3_decoder);
  esp_periph_set_destroy(set);
}

//----------------------------------
static void IRAM_ATTR LED_toggle()
{
#if YMODEM_LED_ACT
  if (GPIO.out & (1 << YMODEM_LED_ACT))
  {
    GPIO.out_w1tc = (1 << YMODEM_LED_ACT);
  }
  else
  {
    GPIO.out_w1ts = (1 << YMODEM_LED_ACT);
  }
#endif
}

static QueueHandle_t uart2_queue;

//---------------------------------------------
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

//------------------------------------------------------------------------
static unsigned short crc16(const unsigned char *buf, unsigned long count)
{
  unsigned short crc = 0;
  int i;

  while (count--)
  {
    crc = crc ^ *buf++ << 8;

    for (i = 0; i < 8; i++)
    {
      if (crc & 0x8000)
        crc = crc << 1 ^ 0x1021;
      else
        crc = crc << 1;
    }
  }
  return crc;
}

//--------------------------------------------------------------
static int32_t Receive_Byte(unsigned char *c, uint32_t timeout)
{
  unsigned char ch;
  int len = uart_read_bytes(EX_UART_NUM, &ch, 1, timeout / portTICK_RATE_MS);
  if (len <= 0)
    return -1;

  *c = ch;
  return 0;
}

//------------------------
static void uart_consume()
{
  uint8_t ch[64];
  while (uart_read_bytes(EX_UART_NUM, ch, 64, 100 / portTICK_RATE_MS) > 0)
    ;
  //    printf("%s", ch, 64);
  // ESP_LOGI(TAG, "%s", ch);
}

//--------------------------------
static uint32_t Send_Byte(char c)
{
  uart_write_bytes(EX_UART_NUM, &c, 1);
  return 0;
}

//----------------------------
static void send_CA(void)
{
  Send_Byte(CA);
  Send_Byte(CA);
}

//-----------------------------
static void send_ACK(void)
{
  Send_Byte(ACK);
}

//----------------------------------
static void send_ACKCRC16(void)
{
  Send_Byte(ACK);
  Send_Byte(CRC16);
}

//-----------------------------
static void send_NAK(void)
{
  Send_Byte(NAK);
}

//-------------------------------
static void send_CRC16(void)
{
  Send_Byte(CRC16);
}

/**
     * @brief  Receive a packet from sender
     * @param  data
     * @param  timeout
     * @param  length
     *    >0: packet length
     *     0: end of transmission
     *    -1: abort by sender
     *    -2: error or crc error
     * @retval 0: normally return
     *        -1: timeout
     *        -2: abort by user
     */
//--------------------------------------------------------------------------
static int32_t Receive_Packet(uint8_t *data, int *length, uint32_t timeout)
{
  int count, packet_size, i;
  unsigned char ch;
  *length = 0;

  // receive 1st byte
  if (Receive_Byte(&ch, timeout) < 0)
  {
    return -1;
  }

  switch (ch)
  {
  case SOH:
    packet_size = PACKET_SIZE;
    ESP_LOGI(TAG, "SOH");
    break;
  case STX:
    packet_size = PACKET_1K_SIZE;
    ESP_LOGI(TAG, "STX");
    break;
  case EOT:
    *length = 0;
    ESP_LOGI(TAG, "EOT");
    return 0;
  case CA:
    if (Receive_Byte(&ch, timeout) < 0)
    {
      return -2;
    }
    if (ch == CA)
    {
      *length = -1;
      return 0;
    }
    else
      return -1;
  case ABORT1:
  case ABORT2:
    return -2;
  default:
    vTaskDelay(100 / portTICK_RATE_MS);
    uart_consume();
    ESP_LOGI(TAG, "UARTCONSUME");
    return -1;
  }

  *data = (uint8_t)ch;
  uint8_t *dptr = data + 1;
  count = packet_size + PACKET_OVERHEAD - 1;

  for (i = 0; i < count; i++)
  {
    if (Receive_Byte(&ch, timeout) < 0)
    {
      return -1;
    }
    *dptr++ = (uint8_t)ch;
    ;
  }

  if (data[PACKET_SEQNO_INDEX] != ((data[PACKET_SEQNO_COMP_INDEX] ^ 0xff) & 0xff))
  {
    ESP_LOGI(TAG, "LENGTH=2");
    *length = -2;
    return 0;
  }
  if (crc16(&data[PACKET_HEADER], packet_size + PACKET_TRAILER) != 0)
  {
    *length = -2;
    ESP_LOGI(TAG, "LENGTH==2");
    return 0;
  }

  *length = packet_size;
  ESP_LOGI(TAG, "LENGTH %d", packet_size);
  return 0;
}

// Receive a file using the ymodem protocol.
//-----------------------------------------------------------------
int Ymodem_Receive(FILE *ffd, unsigned int maxsize, char *getname)
{
  uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD];
  uint8_t *file_ptr;
  char file_size[128];
  unsigned int i, file_len, write_len, session_done, file_done, packets_received, errors, size = 0;
  int packet_length = 0;
  file_len = 0;
  int eof_cnt = 0;
  file_size_ift = 0;

  for (session_done = 0, errors = 0;;)
  {
    ESP_LOGI(TAG, "FOR LOOP:1");
    for (packets_received = 0, file_done = 0;;)
    {
      ESP_LOGI(TAG, "FOR LOOP:2");
      LED_toggle();
      switch (Receive_Packet(packet_data, &packet_length, NAK_TIMEOUT))
      {
      case 0: // normal return
        ESP_LOGI(TAG, "SWTICH STATEMENT:0");
        switch (packet_length)
        {
        case -1:
          // Abort by sender
          ESP_LOGI(TAG, "SWITCH CASE:-1");
          send_ACK();
          size = -1;
          goto exit;
        case -2:
          // error
          errors++;
          ESP_LOGI(TAG, "SWITCH CASE:-2");
          if (errors > 45)
          {
            ESP_LOGI(TAG, "SWITCH CASE:-2, SENDING CA");
            send_CA();
            size = -2;
            goto exit;
          }
          ESP_LOGI(TAG, "SWITCH CASE:-2, SENDING NAK");
          send_NAK();
          break;
        case 0:
          // End of transmission
          eof_cnt++;
          if (eof_cnt == 1)
          {
            ESP_LOGI(TAG, "SWITCH CASE:0 EOT");
            send_NAK();
          }
          else
          {
            ESP_LOGI(TAG, "SWITCH CASE:0 SENDING ACKCRC16");
            send_ACKCRC16();
          }
          break;
        default:
          // ** Normal packet **
          if (eof_cnt > 1)
          {
            ESP_LOGI(TAG, "SENDING ACK IN DEFAULT SWITCH CASE");
            send_ACK();
          }
          else if ((packet_data[PACKET_SEQNO_INDEX] & 0xff) != (packets_received & 0x000000ff))
          {
            errors++;
            ESP_LOGI(TAG, "%d", packet_data[PACKET_SEQNO_INDEX]);
            ESP_LOGI(TAG, "%d", packets_received);
            ESP_LOGI(TAG, "SWITCH CASE DEFAULT ERROR++");
            if (errors > 45)
            {
              ESP_LOGI(TAG, "SENDING CA");
              send_CA();
              size = -3;
              goto exit;
            }
            ESP_LOGI(TAG, "SENDING //NAK//");
            send_NAK();
          }
          else
          {
            if (packets_received == 0)
            {
              // ** First packet, Filename packet **
              if (packet_data[PACKET_HEADER] != 0)
              {
                errors = 0;
                ESP_LOGI(TAG, "%d", packet_data[PACKET_SEQNO_INDEX]);
                ESP_LOGI(TAG, "%d", packets_received);
                // ** Filename packet has valid data
                if (getname)
                {
                  for (i = 0, file_ptr = packet_data + PACKET_HEADER; ((*file_ptr != 0) && (i < 64));)
                  {
                    *getname = *file_ptr++;
                    sprintf(filename_buffer, "%s", getname);
                    getname++;
                  }
                  *getname = '\0';
                }
                for (i = 0, file_ptr = packet_data + PACKET_HEADER; (*file_ptr != 0) && (i < packet_length);)
                {
                  file_ptr++;
                }
                for (i = 0, file_ptr++; (*file_ptr != ' ') && (i < FILE_SIZE_LENGTH);)
                {
                  file_size[i++] = *file_ptr++;
                }
                file_size[i++] = '\0';
                if (strlen(file_size) > 0)
                  size = strtol(file_size, NULL, 10);
                else
                  size = 0;
                printf("The size of the ift is: %d\n", size);
                file_size_ift = size;
                printf("FILESIZE=%ld\n", file_size_ift);
                // Test the size of the file
                if ((size < 1) || (size > maxsize))
                {
                  // End session
                  ESP_LOGI(TAG, "ENDING SESSION");
                  send_CA();
                  if (size > maxsize)
                    size = -9;
                  else
                    size = -4;
                  goto exit;
                }

                file_len = 0;
                ESP_LOGI(TAG, "FILENAME RECEIVED");
                send_ACKCRC16();
              }
              // Filename packet is empty, end session
              else
              {
                errors++;
                if (errors > 5)
                {
                  ESP_LOGI(TAG, "FILENAME PACKET IS EMPTY");
                  send_CA();
                  size = -5;
                  goto exit;
                }
                ESP_LOGI(TAG, "send NAK --");
                send_NAK();
              }
            }
            else
            {
              // ** Data packet **
              // Write received data to file
              if (file_len < size)
              {
                file_len += packet_length; // total bytes received
                if (file_len > size)
                {
                  write_len = packet_length - (file_len - size);
                  file_len = size;
                }
                else
                  write_len = packet_length;

                int written_bytes = fwrite((char *)(packet_data + PACKET_HEADER), 1, write_len, ffd);
                ESP_LOGI(TAG, "THE FILE_WRITE_BYTE_SIZE=%d", written_bytes);

                if (written_bytes != write_len)
                { //failed
                  /* End session */
                  ESP_LOGI(TAG, "ENDING SESSION ");
                  send_CA();
                  size = -6;
                  goto exit;
                }
                //  LED_toggle();
              }
              //success
              errors = 0;
              send_ACK();
            }
            packets_received++;
          }
        }
        break;
      case -2: // user abort
        ESP_LOGI(TAG, "USER ABORT");
        send_CA();
        size = -7;
        goto exit;
      default: // timeout
        if (eof_cnt > 1)
        {
          file_done = 1;
        }
        else
        {
          errors++;
          if (errors > MAX_ERRORS)
          {
            send_CA();
            size = -8;
            goto exit;
          }
          send_CRC16();
        }
      }
      if (file_done != 0)
      {
        session_done = 1;
        break;
      }
    }
    if (session_done != 0)
      break;
  }
exit:
#if YMODEM_LED_ACT
  gpio_set_level(YMODEM_LED_ACT, YMODEM_LED_ACT_ON ^ 1);
#endif
  return size;
}

//------------------------------------------------------------------------------------

void Int2Str(uint8_t *str, int32_t intnum)
{
  uint32_t i, Div = 1000000000, j = 0, Status = 0;
  memset(str, 0, 10);
  for (i = 0; (i < 10) & (Div > 0); i++)
  {
    str[j++] = (intnum / Div) + 48;

    intnum = intnum % Div;
    Div /= 10;
    if ((str[j - 1] == '0') && (Status == 0))
    {
      j = 0;
    }
    else
    {
      Status++;
    }
  }
}

//------------------------------------------------------------------------------------
static void Ymodem_PrepareIntialPacket(uint8_t *data, char *fileName, uint32_t *length)
{
  uint16_t i, j;
  uint8_t file_ptr[10];

  memset(data, 0, PACKET_SIZE + PACKET_HEADER);
  // Make first three packet
  data[0] = SOH;
  data[1] = 0x00;
  data[2] = 0xff;

  /* Filename packet has valid data */
  for (i = 0; (fileName[i] != '\0') && (i < 256); i++)
  {
    data[i + PACKET_HEADER] = fileName[i];
  }

  data[i + PACKET_HEADER] = 0x00;

  Int2Str(file_ptr, *length);

  for (j = 0, i = i + PACKET_HEADER + 1; (file_ptr[j] != '\0') && (j < 10);)
  {
    data[i++] = file_ptr[j++];
  }

  for (j = i; j < PACKET_SIZE + PACKET_HEADER; j++)
  {
    data[j] = 0;
  }
}

//-----------------------------------------------------------------------------------------
static void Ymodem_PreparePacket(uint8_t *data, uint8_t pktNo, uint32_t sizeBlk, FILE *ffd)
{
  uint16_t i, size, packetSize, bytes_read;
  uint16_t tempCRC;

  packetSize = sizeBlk >= PACKET_1K_SIZE ? PACKET_1K_SIZE : PACKET_SIZE;
  size = sizeBlk < packetSize ? sizeBlk : packetSize;

  printf("Inside Prep Packet\n");

  if (packetSize == PACKET_1K_SIZE)
  {
    data[0] = STX;
  }
  else
  {
    data[0] = SOH;
  }

  data[1] = (pktNo);
  data[2] = (~pktNo);

  bytes_read = fread(data + PACKET_HEADER, 1, packetSize, ffd);

  printf("Bytes read %d\n", bytes_read);

  if (size <= packetSize)
  {
    for (i = size + PACKET_HEADER; i < packetSize + PACKET_HEADER; i++)
    {
      data[i] = 0x1A; /* EOF (0x1A) or 0x00 */
    }
  }

  printf("Prepared Packet\n");
}

//-------------------------------------------------------------
static uint8_t Ymodem_WaitResponse(uint8_t ackchr, uint8_t tmo)
{
  unsigned char receivedC;
  uint32_t errors = 0;

  do
  {
    if (Receive_Byte(&receivedC, NAK_TIMEOUT) == 0)
    {
      if (receivedC == ackchr)
      {
        return 1;
      }
      else if (receivedC == CA)
      {
        send_CA();
        return 2; // CA received, Sender abort
      }
      else if (receivedC == NAK)
      {
        return 3;
      }
      else
      {
        return 4;
      }
    }
    else
    {
      errors++;
    }
  } while (errors < tmo);
  return 0;
}

// ---------------------------------------------------------------------
uint16_t UpdateCRC16(uint16_t crcIn, uint8_t byte)
{
  uint32_t crc = crcIn;
  uint32_t in = byte | 0x100;
  do
  {
    crc <<= 1;
    in <<= 1;
    if (in & 0x100)
      ++crc;
    if (crc & 0x10000)
      crc ^= 0x1021;
  } while (!(in & 0x10000));
  return crc & 0xffffu;
}

uint16_t Cal_CRC16(const uint8_t *data, uint32_t size)
{
  uint32_t crc = 0;
  const uint8_t *dataEnd = data + size;
  while (data < dataEnd)
    crc = UpdateCRC16(crc, *data++);

  crc = UpdateCRC16(crc, 0);
  crc = UpdateCRC16(crc, 0);
  return crc & 0xffffu;
}

//------------------------------------------------------------------------
int Ymodem_Transmit(char *sendFileName, unsigned int sizeFile, FILE *ffd)
{
  uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD];
  uint16_t tempCRC, blkNumber;
  FILE *buf_ptr;
  uint8_t tempCheckSum;
  uint8_t receivedC[2], CRC16_F = 0, i;
  int errors;
  uint32_t ackReceived = 0, size = 0, pktSize;

  // Wait for response from receiver
  errors = 0;

  printf("Preparing first packet\n");
  Ymodem_PrepareIntialPacket(packet_data, sendFileName, &sizeFile);
  printf("%s\n%d\n", sendFileName, sizeFile);
  do
  {
    // Send Packet
    uart_flush(EX_UART_NUM);
    uart_write_bytes(EX_UART_NUM, (char *)packet_data, PACKET_SIZE + PACKET_HEADER);

    // Send CRC bytes
    tempCRC = Cal_CRC16(&packet_data[3], PACKET_SIZE);
    Send_Byte(tempCRC >> 8);
    Send_Byte(tempCRC & 0xFF);

    printf("%x\n", tempCRC);

    printf("Sent first packet\n");
    // Wait for Ack
    receivedC[0] = 0;
    if (Receive_Byte(&receivedC[0], 1000) == 0 && (receivedC[0] == CRC16))
    {
      ackReceived = 1;
      printf("Received %c for first packet\n", receivedC[0]);
    }
    else
    {
      printf("Error daww\n");
      errors++;
    }

    printf("%d\n", errors);

  } while (!ackReceived && (errors < 0x0A));

  if (errors >= 0x0A)
  {
    printf("Too many errors - Init Packet\n");
    return errors;
  }

  // === Send file blocks ======================================================
  buf_ptr = ffd;
  size = sizeFile;
  blkNumber = 0x01;

  // Resend packet if NAK  for a count of 10 else end of communication
  while (size)
  {
    // Prepare and send next packet
    uart_flush(EX_UART_NUM);
    printf("BLK Num: %d\n Rem Size: %d\n", blkNumber, size);

    Ymodem_PreparePacket(packet_data, blkNumber, size, ffd);
    ackReceived = 0;
    errors = 0;
    receivedC[0] = 0;
    tempCRC = 0;

    do
    {
      /* Send next packet */
      if (size >= PACKET_1K_SIZE)
      {
        pktSize = PACKET_1K_SIZE;
      }
      else
      {
        pktSize = PACKET_SIZE;
      }

      printf("Prepared next packet %d\n", pktSize);

      uart_write_bytes(EX_UART_NUM, (char *)packet_data, pktSize + PACKET_HEADER);

      tempCRC = Cal_CRC16(&packet_data[3], pktSize);
      Send_Byte(tempCRC >> 8);
      Send_Byte(tempCRC & 0xFF);

      printf("%x\n", tempCRC);

      // Wait for Ack
      if ((Receive_Byte(&receivedC[0], 10000) == 0) && (receivedC[0] == CRC16))
      {
        printf("Received %c\n", receivedC[0]);
        ackReceived = 1;

        if (size > pktSize)
        {
          buf_ptr += pktSize;
          size -= pktSize;
          if (blkNumber == (size / 2))
          {
            return 0xFF; /*  error */
          }
          else
          {
            blkNumber++;
            printf("Moving on\n");
          }
        }
        else
        {
          buf_ptr += pktSize;
          size = 0;
        }
      }
      else
      {
        errors++;
        printf("Error received %c\n", receivedC[0]);
      }
      printf("Errors%d\n", errors);

    } while (!ackReceived && (errors < 0x0A));

    if (errors >= 0x0A)
    {
      return errors;
    }
  }

  ackReceived = 0;
  errors = 0;
  receivedC[0] = 0;
  printf("EOT\n");
  printf("Sending last packet\n");

  do
  {
    // === Send EOT ==============================================================
    Send_Byte(EOT); // Send (EOT)
                    // Wait for Ack

    if ((Receive_Byte(&receivedC[0], 10000) == 0) && receivedC[0] == CRC16)
    {
      ackReceived = 1;
      printf("Got ACK for EOT\n");
    }
    else
    {
      errors++;
    }

  } while (!ackReceived && (errors < 0x0A));

  if (errors >= 0x0A)
  {
    return errors;
  }

  // ackReceived = 0;
  // errors = 0;
  // receivedC[0] = 0;

  // packet_data[0] = SOH;
  // packet_data[1] = 0;
  // packet_data [2] = 0xFF;

  // for (i = PACKET_HEADER; i < (PACKET_SIZE + PACKET_HEADER); i++)
  // {
  //    packet_data [i] = 0x00;
  // }

  //  do
  //    {
  //   // Send Packet
  //   uart_write_bytes(EX_UART_NUM, (char *)packet_data, PACKET_SIZE + PACKET_HEADER);

  //   /* Send CRC or Check Sum based on CRC16_F */
  //   tempCRC = Cal_CRC16(&packet_data[3], PACKET_SIZE);
  //   Send_Byte(tempCRC >> 8);
  //   Send_Byte(tempCRC & 0xFF);

  //   // Wait for Ack
  //     if(Receive_Byte(&receivedC[0], 1000) == 0)
  //     {
  //       if (receivedC[0] == ACK)
  //       {
  //         /* Packet transfered correctly */
  //         ackReceived = 1;
  //         printf("Got ACK for last packet\n");
  //       }
  //     }
  //     else
  //     {
  //         errors++;
  //     }

  //     }while (!ackReceived && (errors < 0x0A));
  //     /* Resend packet if NAK  for a count of 10  else end of commuincation */
  //     if (errors >=  0x0A)
  //     {
  //       return 0x0E;
  //     }

  //   printf("OLT EOT\n");
  //     do
  //     {
  //      // === Send EOT ==============================================================
  //      Send_Byte(EOT); // Send (EOT)
  //      // Wait for Ack

  //       if ((Receive_Byte(&receivedC[0], 1000) == 0)  && receivedC[0] == ACK)
  //       {
  //         ackReceived = 1;
  //       }
  //       else
  //       {
  //         errors++;
  //       }
  //   }while (!ackReceived && (errors < 0x0A));

  // if (errors >=  0x0A)
  // {
  //   return 0x0E;
  // }

  printf("SUCCESS da SK\n");

  return 0; // file transmitted successfully
} //YMODEM PROTOCOL

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
  switch (evt->event_id)
  {
  case HTTP_EVENT_ERROR:
    ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
    break;
  case HTTP_EVENT_ON_CONNECTED:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
    break;
  case HTTP_EVENT_HEADER_SENT:
    ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
    break;
  case HTTP_EVENT_ON_HEADER:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
    break;
  case HTTP_EVENT_ON_DATA:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
    if (!esp_http_client_is_chunked_response(evt->client))
    {
      // Write out data
      switch (http_response_id)
      {
      case PRIVATE_ID:
        memcpy(public_id, evt->data, evt->data_len);
        private_id = strtok(public_id, "|");
        ESP_LOGI(TAG, "HIPER_PRIVATE_ID");
        printf("%s\r\n", private_id);
        break;
      case SESSION_ID:
        memcpy(public_id, evt->data, evt->data_len);
        ESP_LOGI(TAG, "HIPER_SESSION_ID");
        session_id = strtok(public_id, "|");
        printf("%s\r\n", session_id);
        break;
      case LAST_UPLOAD:
        memcpy(public_id, evt->data, evt->data_len);
        ESP_LOGI(TAG, "LASTUPLOADSIZE");
        printf("%.*s", evt->data_len, (char *)evt->data);
        upload_size = strtok(public_id, "|");
        printf("%s\r\n", upload_size);
        break;
      case FILE_UPLOAD:
        printf("%.*s", evt->data_len, (char *)evt->data);
        if ((strcmp((char *)evt->data, SESH_RET)) == 0)
          another_sesh_flag = 1; //                if(evt->data[0]=='S'&& evt->data[8]=='E')
      default:
        break;
      }
      //    printf("%.*s", evt->data_len, (char*)evt->data);
    }

    break;
  case HTTP_EVENT_ON_FINISH:
    ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
    break;
  case HTTP_EVENT_DISCONNECTED:
    ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
    break;
  }
  return ESP_OK;
}

int FILE_DELMOD(void)
{
  int filename_counterx = 0;
  int current_db = 1;
  int i = 1;
  char c[100];
  char ch;
  char upload_filename_temp[25] = "";

  FILE *f_up = fopen(MOUNT_POINT "/filedb.csv", "r");
  if (f_up == NULL)
  {
    ESP_LOGE(TAG, "Failed to open file for reading");
    return 0;
  }
  char upload_filename[25] = "";
  fgets(upload_filename, sizeof(upload_filename), f_up);
  printf("upload done file:%s\n", upload_filename);
  //count no of files
  do
  {
    ch = fgetc(f_up);
    if (ch == '\n')
    {
      ++filename_counterx;
    }
  } while (!feof(f_up));
  fclose(f_up);
  printf("No of remaining files:%d\n", filename_counterx);

  FILE *f_to = fopen(MOUNT_POINT "/TEMP.csv", "w");
  if (f_to == NULL)
  {
    ESP_LOGE(TAG, "Failed to open file for reading");
    return 0; //ERFDB_FXS
  }

  FILE *f_from2 = fopen(MOUNT_POINT "/filedb.csv", "r");
  if (f_from2 == NULL)
  {
    ESP_LOGE(TAG, "Failed to open file for reading");
    return 0;
  }
  //fseek-1 file
  fgets(upload_filename, sizeof(upload_filename), f_from2);

  //do read and write
  for (int i = 0; i <= filename_counterx; i++)
  {
    fgets(upload_filename, sizeof(upload_filename), f_from2);
    if (feof(f_from2))
      break;
    printf("Pending Upload File:%s\n", upload_filename);
    fwrite(upload_filename, 1, 21, f_to);
  }
  fclose(f_from2);
  fclose(f_to);

  // DELETE OLD_FILE
  if (remove("/sdcard/FILEDB.CSV") == 0)
  {
    printf("Deleted Succesfully\n");
  }
  else
  {
    printf("Delete unsuccesful\n");
  }
  //RENAME TEMP_FILE
  if (rename(MOUNT_POINT "/TEMP.csv", MOUNT_POINT "/filedb.csv") != 0)
  {
    ESP_LOGE(TAG, "Rename failed");
    return 0; //ERFDB_FXS
  }

  return filename_counterx;
}

static int waitsocket(int socket_fd, LIBSSH2_SESSION *session)
{
  struct timeval timeout;
  int rc;
  fd_set fd;
  fd_set *writefd = NULL;
  fd_set *readfd = NULL;
  int dir;

  timeout.tv_sec = 10;
  timeout.tv_usec = 0;

  FD_ZERO(&fd);

  FD_SET(socket_fd, &fd);

  /* now make sure we wait in the correct direction */
  dir = libssh2_session_block_directions(session);

  if (dir & LIBSSH2_SESSION_BLOCK_INBOUND)
    readfd = &fd;

  if (dir & LIBSSH2_SESSION_BLOCK_OUTBOUND)
    writefd = &fd;

  rc = select(socket_fd + 1, readfd, writefd, NULL, &timeout);

  return rc;
}
/* TO DOWNLOAD FILES FROM SERVER */
/* Retun 1 -> Failure */
int Server_Download(char *download_filename, FILE *tempstor)
{

  char buffer[1024];

  char sftppath[51] = "";
  // char download_filename[25]="/payload.bin";

  FILE *f_PK_y = fopen(MOUNT_POINT "/PublicID.CSV", "r");

  if (f_PK_y == NULL)
  {
    ESP_LOGE(TAG, "Failed to open publicid for reading");
  }

  fgets(sftppath, 25, f_PK_y);
  fclose(f_PK_y);

  strncat(sftppath, download_filename, 12);
  strncat(sftppath, "\0", 1);

  printf("sftppath:%s\n", (char *)sftppath);

  rc = libssh2_init(0);

  if (rc != 0)
  {
    fprintf(stderr, "libssh2 initialization failed (%d)\n", rc);
    return 1;
  }

  printf("Server Download Start\n");
  /*
   * The application code is responsible for creating the socket
   * and establishing the connection
   */
  sock = socket(AF_INET, SOCK_STREAM, 0);

  sin.sin_family = AF_INET;
  sin.sin_port = htons(1291);
  sin.sin_addr.s_addr = inet_addr(hostaddr);
  if (connect(sock, (struct sockaddr *)(&sin), sizeof(struct sockaddr_in)) != 0)
  {
    fprintf(stderr, "failed to connect!\n");
    return 1; //QUITSOCKET_RET_FXS
  }

  /* Create a session instance
   */
  session = libssh2_session_init();
  if (!session)
  {
    return 1; //QUITSESH_RET_FXS
  }
  printf("Download Session Init\n");

  /* ... start it up. This will trade welcome banners, exchange keys,
   * and setup crypto, compression, and MAC layers
   */

  libssh2_session_set_blocking(session, 1);
  printf("Handshake Init\n");

  while ((rc = libssh2_session_handshake(session, sock)) == LIBSSH2_ERROR_EAGAIN)
  {
    printf("%d\n", rc);
  }
  if (rc)
  {
    fprintf(stderr, "Failure establishing SSH session: %d\n", rc);
    return 1;
  }
  printf("Handshake Success\n");

  /* At this point we havn't yet authenticated.  The first thing to do
   * is check the hostkey's fingerprint against our known hosts Your app
   * may have it hard coded, may go to a file, may present it to the
   * user, that's your call
   */
  fingerprint = libssh2_hostkey_hash(session, LIBSSH2_HOSTKEY_HASH_SHA1);
  fprintf(stderr, "Fingerprint: ");
  for (i = 0; i < 20; i++)
  {
    fprintf(stderr, "%02X ", (unsigned char)fingerprint[i]);
  }
  fprintf(stderr, "\n");

  if (auth_pw)
  {
    /* We could authenticate via password */
    if (libssh2_userauth_password(session, username, password))
    {

      fprintf(stderr, "Authentication by password failed.\n");
      //goto shutdown;
      return 1; //QUIT_AUTH_RET_FXS
    }
  }
  else
  {
    /* Or by public key */
    const char *pubkey = "/home/username/.ssh/id_rsa.pub";
    const char *privkey = "/home/username/.ssh/id_rsa.pub";
    if (libssh2_userauth_publickey_fromfile(session, username, pubkey, privkey, password))
    {

      fprintf(stderr, "\tAuthentication by public key failed\n");
      //goto shutdown;
      return 1;
    }
  }

  fprintf(stderr, "libssh2_sftp_init()!\n");

  do
  {
    sftp_session = libssh2_sftp_init(session);

    if (!sftp_session)
    {
      if (libssh2_session_last_errno(session) == LIBSSH2_ERROR_EAGAIN)
      {
        fprintf(stderr, "non-blocking init\n");
        waitsocket(sock, session); /* now we wait */
      }
      else
      {
        fprintf(stderr, "Unable to init SFTP session\n");
        goto shutdown;
      }
    }

  } while (!sftp_session);

  fprintf(stderr, "libssh2_sftp_open()!\n");

  /* Request a file via SFTP */
  do
  {
    sftp_handle = libssh2_sftp_open(sftp_session, sftppath, LIBSSH2_FXF_READ, 0);

    if (!sftp_handle)
    {
      if (libssh2_session_last_errno(session) != LIBSSH2_ERROR_EAGAIN)
      {

        fprintf(stderr, "Unable to open file with SFTP\n");
        goto shutdown;
      }
      else
      {
        fprintf(stderr, "non-blocking open\n");
        waitsocket(sock, session); /* now we wait */
      }
    }
  } while (!sftp_handle);

  fprintf(stderr, "libssh2_sftp_open() is done, now receive data!\n");

  uart_write_bytes(EX_UART_NUM, "$T202", 5); //AT COMMAND FOR SERVER LOGIN SUCCESSFUL

  audio_playback_handler(SERVER_CONNECT_SUCCESS);

  do
  {
    /* read in a loop until we block */
    rc = libssh2_sftp_read(sftp_handle, buffer, sizeof(buffer));

    fprintf(stderr, "libssh2_sftp_read returned %d\n", rc);

    if (rc > 0)
    {
      /* write to temporary storage area */
      fwrite(buffer, rc, 1, tempstor);
    }

  } while (rc > 0);

  fprintf(stderr, "Download Successfull\n");

  libssh2_sftp_close(sftp_handle);

  fclose(tempstor);

  libssh2_sftp_shutdown(sftp_session);

shutdown:
  libssh2_session_disconnect(session, "Normal Shutdown");
  libssh2_session_free(session);

  close(sock);
  libssh2_exit();

  return 0;
}

/* FILE* server = fopen(MOUNT_POINT"/SERVER.CSV", "w");
  if (server == NULL) 
  {
        ESP_LOGE(TAG, "Failed to open file for reading");
  }
  char temp[] = "BLANK\n0";
  fwrite(temp,1,7,server);

*/

/* Retun 1 -> Failure */
int Server_Upload()
{

  int wifi_connect = 0;
  //TURN ON HOTSPOT AUDIO TRACK
  int ssidint = 0;
  int passint = 0;
  char ssidcc[10] = {};
  char passcc[10] = {};

  FILE *w_cc = fopen(MOUNT_POINT "/wificc.csv", "r");
  if (w_cc == NULL)
  {
    ESP_LOGE(TAG, "Failed to open file for reading");
    return;
  }
  ESP_LOGI(TAG, "get user count\n");
  fgets(ssidcc, 10, w_cc);
  fgets(passcc, 10, w_cc);
  fclose(w_cc);
  sscanf(ssidcc, "%2d", &ssidint);
  sscanf(passcc, "%2d", &passint);
  char USER_SSID[20] = {'\0'};
  char USER_PWD[20] = {'\0'};
  FILE *w_fl = fopen(MOUNT_POINT "/wifissid.csv", "r");
  if (w_fl == NULL)
  {
    ESP_LOGE(TAG, "Failed to open file for reading");
    return;
  }
  ESP_LOGI(TAG, "get user wifi\n");
  fgets(USER_SSID, ssidint + 1, w_fl);
  fclose(w_fl);
  FILE *w_pl = fopen(MOUNT_POINT "/wifipass.csv", "r");
  if (w_pl == NULL)
  {
    ESP_LOGE(TAG, "Failed to open file for reading");
    return;
  }
  ESP_LOGI(TAG, "get user wifi\n");
  fgets(USER_PWD, passint + 1, w_pl);
  fclose(w_pl);
  printf("user:%s\n", USER_SSID);
  printf("pass:%s\n", USER_PWD);

  //  uart_write_bytes(EX_UART_NUM, "$S\n", 1);
  esp_err_t ret1 = nvs_flash_init();
  if (ret1 == ESP_ERR_NVS_NO_FREE_PAGES || ret1 == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret1 = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret1);
  app_wifi_initialise((char *)USER_SSID, (char *)USER_PWD);

  app_wifi_wait_connected();

  printf("Uploading server.csv\n");
  int bytes_read = 0;
  long int file_size;

  char sftppath[51] = "";

  FILE *f_PK_y = fopen(MOUNT_POINT "/PublicID.CSV", "r");
  if (f_PK_y == NULL)
  {
    ESP_LOGE(TAG, "Failed to open file for reading");
    return 1;
  }
  fgets(sftppath, 25, f_PK_y);
  printf("%s\n", (char *)sftppath);
  fclose(f_PK_y);

  char upload_filename[25] = "/server.csv";
  printf("upload filename:%s\n", upload_filename);

  strncat(sftppath, upload_filename, 11);
  strncat(sftppath, "\0", 1);

  printf("sftppath:%s\n", (char *)sftppath);

  FILE *local = fopen(MOUNT_POINT "/server.csv", "r");
  if (local == NULL)
  {
    ESP_LOGE(TAG, "Failed to open file local file for reading");
    fclose(local);
    return 1; ////FILE DOESNT EXIST SO RET ERFDB_FXS
  }

  rc = libssh2_init(0);

  printf("level x1 success\n");
  sock = socket(AF_INET, SOCK_STREAM, 0);

  sin.sin_family = AF_INET;
  sin.sin_port = htons(1291);
  sin.sin_addr.s_addr = inet_addr(hostaddr);
  if (connect(sock, (struct sockaddr *)(&sin),
              sizeof(struct sockaddr_in)) != 0)
  {
    fprintf(stderr, "failed to connect!\n");
    fclose(local);
    return 1; //QUITSOCKET_RET_FXS
  }

  /* Create a session instance
     */
  session = libssh2_session_init();
  printf("level x2 success\n");
  if (!session)
  {
    fclose(local);
    return 1; //QUITSESH_RET_FXS
  }

  /* Since we have set non-blocking, tell libssh2 we are blocking */
  libssh2_session_set_blocking(session, 1);

  printf("level x3 success\n");
  /* ... start it up. This will trade welcome banners, exchange keys,
     * and setup crypto, compression, and MAC layers
     */
  printf("perform client<->server handshake  | now wait_up\n");
  rc = libssh2_session_handshake(session, sock);

  if (rc)
  {
    fclose(local);
    fprintf(stderr, "Failure establishing SSH session: %d\n", rc);
    return 1; //QUITHANDSHAKE_RET_FXS
  }

  printf("level x success\n");
  fingerprint = libssh2_hostkey_hash(session, LIBSSH2_HOSTKEY_HASH_SHA1);

  fprintf(stderr, "Fingerprint: ");
  for (i = 0; i < 20; i++)
  {
    fprintf(stderr, "%02X ", (unsigned char)fingerprint[i]);
  }
  fprintf(stderr, "\n");

  if (auth_pw)
  {
    /* We could authenticate via password */
    if (libssh2_userauth_password(session, username, password))
    {

      fclose(local);
      fprintf(stderr, "Authentication by password failed.\n");
      //goto shutdown;
      return 1; //QUIT_AUTH_RET_FXS
    }
  }
  else
  {
    /* Or by public key */
    const char *pubkey = "/home/username/.ssh/id_rsa.pub";
    const char *privkey = "/home/username/.ssh/id_rsa.pub";
    if (libssh2_userauth_publickey_fromfile(session, username,

                                            pubkey, privkey,
                                            password))
    {
      fprintf(stderr, "\tAuthentication by public key failed\n");
      //goto shutdown;
      return 1;
    }
  }

  fprintf(stderr, "libssh2_sftp_init()!\n");

  sftp_session = libssh2_sftp_init(session);

  if (!sftp_session)
  {
    fprintf(stderr, "Unable to init SFTP session\n");
    fclose(local);
    //goto shutdown;
    return 1;
  }

  fprintf(stderr, "libssh2_sftp_open()!\n");

  sftp_handle =
      libssh2_sftp_open(sftp_session, (char *)sftppath, LIBSSH2_FXF_WRITE | LIBSSH2_FXF_CREAT | LIBSSH2_FXF_TRUNC,
                        LIBSSH2_SFTP_S_IRUSR | LIBSSH2_SFTP_S_IWUSR |
                            LIBSSH2_SFTP_S_IRGRP | LIBSSH2_SFTP_S_IROTH);

  if (!sftp_handle)
  {
    fprintf(stderr, "Unable to open file with SFTP\n");
    fclose(local);
    return 1;
    //goto shutdown;
  }
  fprintf(stderr, "libssh2_sftp_open() is done, now send data!\n");
  int i_x = 0;
  do
  {
    nread = fread(mem, 1, sizeof(mem), local);
    //printf("%s",(char*)mem);
    //nread is no.of bytes successfully read by fread
    if (nread <= 0)
    {
      /* end of file */
      break;
    }
    ptr = mem;

    do
    {
      /* write data in a loop until we block */
      rc = libssh2_sftp_write(sftp_handle, ptr, nread); //rc - Actual number of bytes written or negative on failure.
      printf("\nProgress:%d\n", ++i_x);
      //uart_write_bytes(EX_UART_NUM, "#", 1);
      if (rc < 0)
      {
        printf("break\n");
        //  uart_write_bytes(EX_UART_NUM, "@", 1);
        break;
      }
      ptr += rc;
      nread -= rc; //remaining to upload of the 1KB
    } while (nread);

  } while (rc > 0);

  fclose(local);

  if (nread <= 0)
  {
    printf("\nUpload Done\n");
  }

  else
    printf("\nUpload Incomplete\n");

  libssh2_sftp_close(sftp_handle);

  libssh2_sftp_shutdown(sftp_session);

  //shutdown:
  libssh2_session_disconnect(session, "Normal Shutdown, Thank you for playing");
  libssh2_session_free(session);

  close(sock);

  libssh2_exit();

  printf("\nDone uploading\n");

  return 0;
}
/*TO DOWNLOAD SERVER.CSV FILE*/
int Server_Connect()
{

  char download_filename[] = "/server.csv";
  int wifi_connect = 0;
  //TURN ON HOTSPOT AUDIO TRACK
  audio_playback_handler(hotspotreq);
  int ssidint = 0;
  int passint = 0;
  char ssidcc[10] = {};
  char passcc[10] = {};

  FILE *w_cc = fopen(MOUNT_POINT "/wificc.csv", "r");
  if (w_cc == NULL)
  {
    ESP_LOGE(TAG, "Failed to open wificc.csv for reading");
    return;
  }
  ESP_LOGI(TAG, "get user count\n");
  fgets(ssidcc, 10, w_cc);
  fgets(passcc, 10, w_cc);
  fclose(w_cc);
  sscanf(ssidcc, "%2d", &ssidint);
  sscanf(passcc, "%2d", &passint);
  char USER_SSID[20] = {'\0'};
  char USER_PWD[20] = {'\0'};
  FILE *w_fl = fopen(MOUNT_POINT "/wifissid.csv", "r");
  if (w_fl == NULL)
  {
    ESP_LOGE(TAG, "Failed to open wifissid.csv for reading");
    return;
  }
  ESP_LOGI(TAG, "get user wifi\n");
  fgets(USER_SSID, ssidint + 1, w_fl);
  fclose(w_fl);
  FILE *w_pl = fopen(MOUNT_POINT "/wifipass.csv", "r");
  if (w_pl == NULL)
  {
    ESP_LOGE(TAG, "Failed to open wifipass.csv for reading");
    return;
  }
  ESP_LOGI(TAG, "get user wifi\n");
  fgets(USER_PWD, passint + 1, w_pl);
  fclose(w_pl);
  printf("user:%s\n", USER_SSID);
  printf("pass:%s\n", USER_PWD);

  esp_err_t ret1 = nvs_flash_init();
  if (ret1 == ESP_ERR_NVS_NO_FREE_PAGES || ret1 == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret1 = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret1);
  app_wifi_initialise((char *)USER_SSID, (char *)USER_PWD);

  app_wifi_wait_connected();
  //AT COMMAND FOR HOTPSPOT CONNECTED
  uart_write_bytes(EX_UART_NUM, "$T201", 5);
  //PLAYBACK HOTSPOT_CONNECTED
  audio_playback_handler(hotspoton);

  char update[20];

  FILE *tempst = fopen(MOUNT_POINT "/server.csv", "wb");
  if (!tempst)
  {
    fprintf(stderr, "Can't open temp storage file SERVER.CSV\n");
  }

  wifi_connect = Server_Download(download_filename, tempst);

  printf("Wifi Connect %d\n", wifi_connect);

  /******* Server.csv Download complete **********/

  return wifi_connect;
}
/* FUNCTION FOR SFTP UPLOAD TO SERVER */
int SFTPCurl(void)
{
  /*Error returning conditions:
    -> SFTP_Upload path File(PublicID.CSV) not found                              | 
    -> Filedatabase not found
    -> Filedatabase empty/not an error but no files to upload
    -> Filedabase modify error [try to save FDB.CSV]
    -> Local to upload file not present
    -> SFTP ERRORS:
     => SFTP Init error             | doesnt return
     => SFTP Socket error           | returns -> redo function from start
     => SFTP Session Init_1 error   | returns -> redo from start
     => SFTP Handshake error        | returns -> redo from start
     => SFTP Session Init_2 error   | returns -> redo from start
     => SFTP Path Error             | returns -> fix path
     => SFTP Upload/Put/Write error | returns -> put back fn to fdb and exit
     =>  
    */
  int filename_counter = 1; // DO_ALL_FILES+1
  do
  {

    int bytes_read = 0;
    long int file_size;

    char sftppath[51] = "";

    FILE *f_PK_y = fopen(MOUNT_POINT "/PublicID.CSV", "r");
    if (f_PK_y == NULL)
    {
      ESP_LOGE(TAG, "Failed to open publicid.csv for reading");
    }
    fgets(sftppath, 25, f_PK_y);
    printf("%s\n", (char *)sftppath);
    fclose(f_PK_y);

    //    FILEDATABASE.CSV HANDLING --> ERFDB_XX     //
    // NO_FILE -1
    FILE *f_up = fopen(MOUNT_POINT "/filedb.csv", "r");
    if (f_up == NULL)
    {
      ESP_LOGE(TAG, "Failed to open filedb.csv for reading");
      return -1;
    }
    // GET_LOCALFN
    // GET_NO_FILES
    // GET_REM_FILES
    char upload_filename[25] = "";
    fgets(upload_filename, sizeof(upload_filename), f_up);
    printf("upload filename:%s\n", upload_filename);
    fclose(f_up);

    // UPLOAD FILE CHECK ERNOLF
    FILE *local = fopen(upload_filename, "r");
    if (local == NULL)
    {
      ESP_LOGE(TAG, "Failed to open file local file for reading");
      fclose(local);
      return filename_counter; ////FILE DOESNT EXIST SO RET ERFDB_FXS
    }

    strncat(sftppath, upload_filename, 20);
    strncat(sftppath, "\0", 1);

    printf("sftppath:%s\n", (char *)sftppath);

    //GETSIZE

    //          STP_ERROR -> QUIT_RET_FXS
    //          SFTP_CONT_RETRY   -> CTRY
    rc = libssh2_init(0);

    printf("level x1 success\n");
    /*
     * The application code is responsible for creating the socket
     * and establishing the connection
     */
    sock = socket(AF_INET, SOCK_STREAM, 0);

    sin.sin_family = AF_INET;
    sin.sin_port = htons(1291);
    sin.sin_addr.s_addr = inet_addr(hostaddr);
    if (connect(sock, (struct sockaddr *)(&sin),
                sizeof(struct sockaddr_in)) != 0)
    {
      fprintf(stderr, "failed to connect!\n");
      fclose(local);
      return filename_counter; //QUITSOCKET_RET_FXS
    }

    /* Create a session instance
     */
    session = libssh2_session_init();
    printf("level x2 success\n");
    if (!session)
    {
      fclose(local);
      return filename_counter; //QUITSESH_RET_FXS
    }

    /* Since we have set non-blocking, tell libssh2 we are blocking */
    libssh2_session_set_blocking(session, 1);

    printf("level x3 success\n");
    /* ... start it up. This will trade welcome banners, exchange keys,
     * and setup crypto, compression, and MAC layers
     */
    printf("perform client<->server handshake  | now wait_up\n");
    rc = libssh2_session_handshake(session, sock);

    if (rc)
    {
      fclose(local);
      fprintf(stderr, "Failure establishing SSH session: %d\n", rc);
      return filename_counter; //QUITHANDSHAKE_RET_FXS
    }

    printf("level x success\n");
    fingerprint = libssh2_hostkey_hash(session, LIBSSH2_HOSTKEY_HASH_SHA1);

    fprintf(stderr, "Fingerprint: ");
    for (i = 0; i < 20; i++)
    {
      fprintf(stderr, "%02X ", (unsigned char)fingerprint[i]);
    }
    fprintf(stderr, "\n");

    if (auth_pw)
    {
      /* We could authenticate via password */
      if (libssh2_userauth_password(session, username, password))
      {

        fclose(local);
        fprintf(stderr, "Authentication by password failed.\n");
        //goto shutdown;
        return filename_counter; //QUIT_AUTH_RET_FXS
      }
    }
    else
    {
      /* Or by public key */
      const char *pubkey = "/home/username/.ssh/id_rsa.pub";
      const char *privkey = "/home/username/.ssh/id_rsa.pub";
      if (libssh2_userauth_publickey_fromfile(session, username,

                                              pubkey, privkey,
                                              password))
      {
        fprintf(stderr, "\tAuthentication by public key failed\n");
        //goto shutdown;
        return filename_counter;
      }
    }

    fprintf(stderr, "libssh2_sftp_init()!\n");

    sftp_session = libssh2_sftp_init(session);

    if (!sftp_session)
    {
      fprintf(stderr, "Unable to init SFTP session\n");
      fclose(local);
      //goto shutdown;
      return filename_counter;
    }

    fprintf(stderr, "libssh2_sftp_open()!\n");

    sftp_handle =
        libssh2_sftp_open(sftp_session, (char *)sftppath, LIBSSH2_FXF_WRITE | LIBSSH2_FXF_CREAT | LIBSSH2_FXF_TRUNC,
                          LIBSSH2_SFTP_S_IRUSR | LIBSSH2_SFTP_S_IWUSR |
                              LIBSSH2_SFTP_S_IRGRP | LIBSSH2_SFTP_S_IROTH);

    if (!sftp_handle)
    {
      fprintf(stderr, "Unable to open file with SFTP\n");
      fclose(local);
      return filename_counter;
      //goto shutdown;
    }

    uart_write_bytes(EX_UART_NUM, "$T102", 5); //ESP LOGGED IN SFTP SERVER

    fprintf(stderr, "libssh2_sftp_open() is done, now send data!\n");
    int i_x = 0;
    do
    {
      nread = fread(mem, 1, sizeof(mem), local);
      //printf("%s",(char*)mem);
      //nread is no.of bytes successfully read by fread
      if (nread <= 0)
      {
        /* end of file */
        break;
      }
      ptr = mem;

      do
      {
        /* write data in a loop until we block */
        rc = libssh2_sftp_write(sftp_handle, ptr, nread); //rc - Actual number of bytes written or negative on failure.
        printf("\nProgress:%d\n", ++i_x);
        //uart_write_bytes(EX_UART_NUM, "#", 1);
        if (rc < 0)
        {
          printf("break\n");
          //  uart_write_bytes(EX_UART_NUM, "@", 1);
          break;
        }
        ptr += rc;
        nread -= rc; //remaining to upload of the 1KB
      } while (nread);

    } while (rc > 0);

    fclose(local);

    if (nread <= 0)
    {
      printf("\nUpload Done\n");
      filename_counter = FILE_DELMOD();
    }

    else
      printf("\nUpload Incomplete\n");

    libssh2_sftp_close(sftp_handle);

    libssh2_sftp_shutdown(sftp_session);

    //shutdown:
    libssh2_session_disconnect(session,
                               "Normal Shutdown, Thank you for playing");
    libssh2_session_free(session);

    close(sock);

    libssh2_exit();

    uart_write_bytes(EX_UART_NUM, "$T103", 5); //SEND CHARACTER FOR EVERY FILE UPLOAD
  } while (filename_counter != 0);
  printf("\nAll files done uploading\n");
  return filename_counter;
}

FILE *IFT_file;
FILE *tempstorage;
char IFT_filename[25];
long file_alavu = 0;
int result, IFT_start = 0, error_IFT = 0;
char download_filename[25];

char pay[] = "/payload.bin";
char DBC[] = "/dbc.csv";
char cmd[] = "/cmd.csv";
char ECU[] = "/ECU.bin";
char STM[] = "/STM.bin";
char DTCBIN[] = "/DTC.bin";
char DTCCMD[] = "/dtc.csv";
char ESP[] = "/esp.bin";

void rx_task_2()
{
  static const char *RX_TASK_TAG = "RX_TASK";
  esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
  uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
  char time[7];
  char lat[11];
  char lon[11];
  char u_id[25];
  char v_cd[7];
  char v_no[13];
  char *q = (char *)u_id;
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
        ESP_LOGI(TAG, "INTO STM COMMANDS");

        //STM TELLS ESP TO GO TO BOOTLOADER

        if ((data[1] == 'B') && (data[2] == '1'))
        {
          reboot_esp(); //GO TO BOOTLOADER CODE
        }

        /*PLAY WELCOME AUDIO ($M01)*/
        if ((data[1] == 'M') && (data[2] == '0') && (data[2] == '1'))
        {
          ESP_LOGI(AT, "$N001");
          audio_playback_handler(welcome);
          uart_write_bytes(EX_UART_NUM, "$N001", 6);
        }

        /* DEVICE STATUS PING READY */
        if ((data[1] == 'P') && (data[2] == '1'))
        {
          uart_write_bytes(EX_UART_NUM, "$Q101", 6);
        }

        /* DEVICE STATUS PING TO SERVER */

        if ((data[1] == 'P') && (data[2] == '2'))
        {
          char url[450] = "https://hiperautomotive.com/heartbeatcheck.php?";
          int j = 0;
          //MESSAGE TYPE
          printf("Message type: %x\n", data[3]);

          /*RTC PING TO SERVER*/
          ESP_LOGI(AT, "RTC PING TO BE SENT");
          if (data[3] == '0')
          {
            //DATE&TIME
            for (int i = 4; i < 10; i++)
            {
              time[j] = data[i];
              j++;
            }
            time[j] = '\0';
            j = 0;

            printf("Time :%s\n", time);
            //LATITUDE
            for (int k = 11; k < 21; k++)
            {
              lat[j] = data[k];
              j++;
            }
            lat[j] = '\0';
            j = 0;

            printf("LATITUDE :%s\n", lat);
            //LONGITUDE
            for (int l = 22; l < 32; l++)
            {
              lon[j] = data[l];
              j++;
            }
            lon[j] = '\0';
            printf("LONGITUDE :%s\n", lon);
            j = 0;
            //UNIQUE ID OF STM PROCESSOR
            for (int p = 33; p <= rxBytes; p++)
            {
              u_id[j] = data[p];
              j++;
            }
            u_id[j] = '\0';

            printf("UNQIUE ID :%s\n", u_id);

            strcat(url, "M_TYPE=");
            strcat(url, "0");

            strcat(url, "&LAT=");
            strncat(url, lat, 10);
            strcat(url, "&LON=");
            strncat(url, lon, 10);

            strcat(url, "&TIM=");
            //strncat(url, time, 6);
            strncat(url, &time[0], 1);
            strncat(url, &time[1], 1);
            strcat(url, ":");
            strncat(url, &time[2], 1);
            strncat(url, &time[3], 1);
            strcat(url, ":");
            strncat(url, &time[4], 1);
            strncat(url, &time[5], 1);
            uart_write_bytes(EX_UART_NUM, "$Q201", 6);
          }

          /*IGNITION WAKEUP*/
          if (data[3] == '1')
          {
            ESP_LOGI(AT, "IGNITION WAKEUP PING TO BE SENT");
            //DATE&TIME
            for (int i = 4; i < 10; i++)
            {
              time[j] = data[i];
              j++;
            }
            time[j] = '\0';
            j = 0;

            printf("Time :%s\n", time);

            for (int p = 11; p <= rxBytes; p++)
            {
              u_id[j] = data[p];
              j++;
            }
            u_id[j] = '\0';

            printf("UNQIUE ID :%s\n", u_id);

            //RTC HEARTBEAT URL

            strcat(url, "M_TYPE=");
            strcat(url, "1");
            strcat(url, "&TIM=");
            //strncat(url, time, 6);
            strncat(url, &time[0], 1);
            strncat(url, &time[1], 1);
            strcat(url, ":");
            strncat(url, &time[2], 1);
            strncat(url, &time[3], 1);
            strcat(url, ":");
            strncat(url, &time[4], 1);
            strncat(url, &time[5], 1);
            uart_write_bytes(EX_UART_NUM, "$Q201", 6);
          }

          /*IWDG ERROR WAKEUP*/
          if (data[3] == '2')
          {
            ESP_LOGI(AT, "IWDG ERROR WAKEUP PING TO BE SENT");
            //DATE&TIME
            for (int i = 4; i < 10; i++)
            {
              time[j] = data[i];
              j++;
            }
            time[j] = '\0';
            j = 0;

            printf("Time :%s\n", time);

            for (int p = 11; p <= rxBytes; p++)
            {
              u_id[j] = data[p];
              j++;
            }
            u_id[j] = '\0';

            printf("UNQIUE ID :%s\n", u_id);

            //RTC HEARTBEAT URL

            strcat(url, "M_TYPE=");
            strcat(url, "2");
            strcat(url, "&TIM=");
            //strncat(url, time, 6);
            strncat(url, &time[0], 1);
            strncat(url, &time[1], 1);
            strcat(url, ":");
            strncat(url, &time[2], 1);
            strncat(url, &time[3], 1);
            strcat(url, ":");
            strncat(url, &time[4], 1);
            strncat(url, &time[5], 1);
            uart_write_bytes(EX_UART_NUM, "$Q201", 6);
          }

          /*UPDATE CHECKUP WAKEUP*/
          if (data[3] == '3')
          {
            ESP_LOGI(AT, "UPDATE CHECKUP WAKEUP PING TO BE SENT");
            for (int i = 4; i < 10; i++)
            {
              time[j] = data[i];
              j++;
            }
            time[j] = '\0';
            j = 0;

            printf("Time :%s\n", time);
           

            //VERSION NUMBER
            for (int l = 11; l < 21; l++)
            {
              v_no[j] = data[l];
              j++;
            }
            v_no[j] = '\0';
            printf("VERSION NO. :%s\n", v_no);
            j = 0;
            //UNIQUE ID OF STM PROCESSOR
            for (int p = 22; p < rxBytes; p++)
            {
              u_id[j] = data[p];
              j++;
            }
            u_id[j] = '\0';

            printf("UNQIUE ID :%s\n", u_id);

            strcat(url, "M_TYPE=");
            strcat(url, "3");
            // strcat(url, "&VerCode=");
            // strncat(url, v_cd, 6);
            strcat(url, "&VerNo=");
            strncat(url, v_no, 10);
            strcat(url,"0");
            strncat(url,&VERSION_NO[0],1);
            strcat(url, "&TIM=");
            //strncat(url, time, 6);

            strncat(url, &time[0], 1);
            strncat(url, &time[1], 1);
            strcat(url, ":");
            strncat(url, &time[2], 1);
            strncat(url, &time[3], 1);
            strcat(url, ":");
            strncat(url, &time[4], 1);
            strncat(url, &time[5], 1);
            uart_write_bytes(EX_UART_NUM, "$Q201", 6);
          }

          if (data[0] == '$')
          {
            const char *pos_data = q;

            /* WIFI FILE OPENING AND CONNECTION */

            int ssidint = 0;
            int passint = 0;
            char ssidcc[10] = {};
            char passcc[10] = {};

            FILE *w_cc = fopen(MOUNT_POINT "/wificc.csv", "r");
            audio_playback_handler(hotspotreq); //turn on hotspot required
            if (w_cc == NULL)
            {
              ESP_LOGE(TAG, "Failed to open file for reading");
              return;
            }
            ESP_LOGI(TAG, "get user count\n");
            fgets(ssidcc, 10, w_cc);
            fgets(passcc, 10, w_cc);
            fclose(w_cc);
            sscanf(ssidcc, "%2d", &ssidint);
            sscanf(passcc, "%2d", &passint);
            char USER_SSID[20] = {'\0'};
            char USER_PWD[20] = {'\0'};
            FILE *w_fl = fopen(MOUNT_POINT "/wifissid.csv", "r");
            if (w_fl == NULL)
            {
              ESP_LOGE(TAG, "Failed to open file for reading");
              return;
            }
            ESP_LOGI(TAG, "get user wifi\n");
            fgets(USER_SSID, ssidint + 1, w_fl);
            fclose(w_fl);
            FILE *w_pl = fopen(MOUNT_POINT "/wifipass.csv", "r");
            if (w_pl == NULL)
            {
              ESP_LOGE(TAG, "Failed to open file for reading");
              return;
            }
            ESP_LOGI(TAG, "get user wifi\n");
            fgets(USER_PWD, passint + 1, w_pl);
            fclose(w_pl);
            printf("ssid:%s\n", USER_SSID);
            printf("pass%s\n", USER_PWD);
            vTaskDelay(1000 / portTICK_RATE_MS);

            esp_err_t ret = nvs_flash_init();
            if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
            {
              ESP_ERROR_CHECK(nvs_flash_erase());
              ret = nvs_flash_init();
            }

            ESP_ERROR_CHECK(ret);
            app_wifi_initialise((char *)USER_SSID, (char *)USER_PWD);

            app_wifi_wait_connected();

            uart_write_bytes(EX_UART_NUM, "$Q202", 6); //CONNECTED TO HOTSPOT
            audio_playback_handler(hotspoton);         //hotspot connected audio
            char *p = (char *)url;
            printf("RTC HEARTBEAT URL :%s", (char *)p);
            const char *rtc_url = p;
            ESP_LOGI(TAG, "Connected to AP, begin http example");
            /* RTC HEARTBEAT POST HTTPS REQUEST */
            esp_http_client_config_t config = {
                .url = "http://httpbin.org/get",
                .event_handler = _http_event_handler,
            };
            esp_http_client_handle_t client = esp_http_client_init(&config);

            esp_http_client_set_header(client, "DEVICEID", u_id);
            esp_http_client_set_url(client, rtc_url); //(char*)register_url);
            esp_http_client_set_method(client, HTTP_METHOD_POST);
            esp_http_client_set_post_field(client, pos_data, strlen(pos_data));
            esp_err_t err = esp_http_client_perform(client);
            if (err == ESP_OK)
            {
              ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                       esp_http_client_get_status_code(client),
                       esp_http_client_get_content_length(client));
              if (esp_http_client_get_status_code(client) == 200)
              {
                uart_write_bytes(EX_UART_NUM, "$Q203", 6);
              }
              else
              {
                //SAVE DATA LOCALLY IF REQUEST FAILS
                // ESP_LOGI(TAG, "RTC HEARTBEAT LOCAL");
                // ESP_LOGI(TAG, "Opening file");

                // FILE *f_stat = fopen(MOUNT_POINT "/RTC.csv", "a+");
                // if (f_stat == NULL)
                // {
                //   ESP_LOGE(TAG, "Failed to open RTC.CSV for reading");
                // }
                // printf("TIME :%s\n", time);
                // fwrite((char *)(time), 1, strlen(time), f_stat);
                // fwrite(",", 1, 1, f_stat);
                // fwrite((char *)(lat), 1, strlen(lat), f_stat);
                // fwrite(",", 1, 1, f_stat);
                // fwrite((char *)(lon), 1, strlen(lon), f_stat);
                // fwrite(",", 1, 1, f_stat);
                // fwrite("HTTP STATUS: ", 1, 14, f_stat);
                // fwrite(esp_http_client_get_status_code(client), 1, 3, f_stat);
                // fwrite("\n", 1, 1, f_stat);
                // fclose(f_stat);
                uart_write_bytes(EX_UART_NUM, "$Q204", 6);
              }
            }
            //SAVE DATA LOCALLY IF REQUEST FAILS
            else
            {
              // ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
              // ESP_LOGI(TAG, "RTC HEARTBEAT LOCAL");
              // ESP_LOGI(TAG, "Opening file");

              // FILE *f_rt = fopen(MOUNT_POINT "/RTC.csv", "a+");
              // if (f_rt == NULL)
              // {
              //   ESP_LOGE(TAG, "Failed to open RTC.CSV for reading");
              // }
              // fwrite((char *)(time), 1, strlen(time), f_rt);
              // fwrite(",", 1, 1, f_rt);
              // fwrite((char *)(lat), 1, strlen(lat), f_rt);
              // fwrite(",", 1, 1, f_rt);
              // fwrite((char *)(lon), 1, strlen(lon), f_rt);
              // fwrite(",", 1, 1, f_rt);
              // fwrite("FAIL REQUEST", 1, 13, f_rt);
              // fwrite("\n", 1, 1, f_rt);
              // fclose(f_rt);

              uart_write_bytes(EX_UART_NUM, "$Q205", 16);
            }
            esp_http_client_cleanup(client);
          }
          else if (data[0] == '*')
          {
            FILE *frt = fopen(MOUNT_POINT "/RTC.csv", "a+");
            if (frt == NULL)
            {
              ESP_LOGE(TAG, "Failed to open RTC.CSV for reading");
            }
            printf("TIME :%s\n", time);
            fwrite((char *)(time), 1, strlen(time), frt);
            fwrite(",", 1, 1, frt);
            fwrite((char *)(lat), 1, strlen(lat), frt);
            fwrite(",", 1, 1, frt);
            fwrite((char *)(lon), 1, strlen(lon), frt);
            fwrite(",", 1, 1, frt);
            fwrite("NO WIFI", 1, 8, frt);
            fwrite("\n", 1, 1, frt);
            fclose(frt);
          }
        }

        /* TO SEE IF FILES ARE THEIR TO BE UPLOADED TO SERVER */
        else if ((data[1] == 'A') && (data[2] == '1'))
        {
          ESP_LOGI(AT, "$A1");
          printf("CHECKING FOR FILE UPLOAD\n");
          /* FILE WHICH KEEP THE NAME OF FILES TO BE UPLOADED */
          FILE *f_UPxs = fopen(MOUNT_POINT "/filedb.csv", "r");
          if (f_UPxs == NULL)
          {
            ESP_LOGE(TAG, "Failed to open filedb.csv for reading");
            uart_write_bytes(EX_UART_NUM, "%", 1);
            return 0;
          }
          char filexs;
          int upflcnt = 0;
          //count no of files
          do
          {
            filexs = fgetc(f_UPxs);
            if (filexs == '\n')
            {
              ++upflcnt;
            }
          } while (!feof(f_UPxs));
          fclose(f_UPxs);
          printf("No of remaining files:%d\n", upflcnt);
          if (upflcnt == 0)
          {
            /* NO LOG FILES TO UPLOAD TO SERVER */
            uart_write_bytes(EX_UART_NUM, "$B102", 6);
          }
          else
          {
            /* THIER ARE FILES TO UPLOAD TO SERVER */
            uart_write_bytes(EX_UART_NUM, "$B101", 4);
          }
        }
        /* UPLOADING LOG FILES TO SERVER */
        else if (data[1] == 'S' && data[2] == '1')
        {
          ESP_LOGI(AT, "$S1");
          printf("UPLOADING LOG FILES\n");

          //TURN ON HOTSPOT AUDIO TRACK
          audio_playback_handler(hotspotreq);
          int ssidint = 0;
          int passint = 0;
          char ssidcc[10] = {};
          char passcc[10] = {};

          FILE *w_cc = fopen(MOUNT_POINT "/wificc.csv", "r");
          if (w_cc == NULL)
          {
            ESP_LOGE(TAG, "Failed to open wificc.csv for reading");
            return;
          }
          ESP_LOGI(TAG, "get user count\n");
          fgets(ssidcc, 10, w_cc);
          fgets(passcc, 10, w_cc);
          fclose(w_cc);
          sscanf(ssidcc, "%2d", &ssidint);
          sscanf(passcc, "%2d", &passint);
          char USER_SSID[20] = {'\0'};
          char USER_PWD[20] = {'\0'};
          FILE *w_fl = fopen(MOUNT_POINT "/wifissid.csv", "r");
          if (w_fl == NULL)
          {
            ESP_LOGE(TAG, "Failed to open wifissid.csv for reading");
            return;
          }
          ESP_LOGI(TAG, "get user wifi\n");
          fgets(USER_SSID, ssidint + 1, w_fl);
          fclose(w_fl);
          FILE *w_pl = fopen(MOUNT_POINT "/wifipass.csv", "r");
          if (w_pl == NULL)
          {
            ESP_LOGE(TAG, "Failed to open wifipass.csv for reading");
            return;
          }
          ESP_LOGI(TAG, "get user wifi\n");
          fgets(USER_PWD, passint + 1, w_pl);
          fclose(w_pl);
          printf("user:%s\n", USER_SSID);
          printf("pass:%s\n", USER_PWD);

          esp_err_t ret1 = nvs_flash_init();
          if (ret1 == ESP_ERR_NVS_NO_FREE_PAGES || ret1 == ESP_ERR_NVS_NEW_VERSION_FOUND)
          {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret1 = nvs_flash_init();
          }

          ESP_ERROR_CHECK(ret1);
          app_wifi_initialise((char *)USER_SSID, (char *)USER_PWD);

          app_wifi_wait_connected();
          ESP_LOGI(TAG, "Connected to AP, begin http example");
          wifi_already_on = 1;
          uart_write_bytes(EX_UART_NUM, "$T101", 5); //AT COMMAND FOR HOTSPOT CONNECTED
          //PLAYBACK HOTSPOT_CONNECTED
          audio_playback_handler(hotspoton);

          // //Server Download
          // int ret_down;
          // ret_down = Server_Download();

          /* SFTP UPLOAD STARTS HERE */
          int SFTP_RUNNER_VAL = 1;
          int SFTP_TRIALS = 5;
          int m = 0;
          do
          {
            SFTP_RUNNER_VAL = SFTPCurl();
            --SFTP_TRIALS;
            printf("\nSFTP_TRIAL:%d\n", SFTP_TRIALS);
            if (m > 0)
            {
              uart_write_bytes(EX_UART_NUM, "$T104", 5);
            }
            m++;
          } while (SFTP_RUNNER_VAL != 0 && SFTP_TRIALS > 0);
          // if (SFTP_RUNNER_VAL != 0)
          // {
          //   printf("IF PART\n");
          //   uart_write_bytes(EX_UART_NUM, "@", 1);
          // }
          // else
          // {
          printf("ELSE PART\n");
          audio_playback_handler(hotspotoff);
          printf("Completed Server Download");
          uart_write_bytes(EX_UART_NUM, "$T105", 5);
          //PLAYBACK TURN OFF HOTSPOT
          //}
          //xTaskCreatePinnedToCore(&SFTPCurl, "SFTPCurl", 10*1024, NULL, 5, NULL, tskNO_AFFINITY);
        }
        /* DOWNLOAD SERVER.CSV AND CHECK FOR UPDATES AND DOWNLOAD THEM */
        else if ((data[1] == 'S') && (data[2] == '2'))
        {
          ESP_LOGI(AT, "$S2");
          printf("CHECKING FOR UPDATES\n");
          int connection;
          uint16_t update_status = 0x00;
          connection = Server_Connect(); //DOWNLOADING SERVER.CSV

          if (connection == 0)
          {
            uart_write_bytes(EX_UART_NUM, "$T203\n", 1); //AT COMMAND FOR SUCCESSFUL DOWNLOAD

            /******* Check updates in server.csv  **********/
            char update[25] = {'\0'};
            char pay_upd[4] = {'\0'};
            char dbc_upd[4] = {'\0'};
            char cmd_upd[4] = {'\0'};
            char ecu_upd[4] = {'\0'};
            char stm_upd[4] = {'\0'};
            char st_cmd[10] = {'\0'};
            char esp_upd[4] = {'\0'};
            FILE *check_up = fopen(MOUNT_POINT "/server.csv", "r+");
            fgets(update, 20, check_up);
            memset(update, 0, 20);

            fgets(update, 20, check_up);
            printf("payload.bin version update : %s\n", update);
            strcpy(pay_upd, update);
            fgets(update, 20, check_up);
            printf("dbc.csv version update : %s\n", update);
            strcpy(dbc_upd, update);
            memset(update, 0, 20);
            fgets(update, 20, check_up);
            printf("cmd.csv version update : %s\n", update);
            strcpy(cmd_upd, update);
            memset(update, 0, 20);
            fgets(update, 20, check_up);
            printf("ecu.bin version update : %s\n", update);
            strcpy(ecu_upd, update);
            memset(update, 0, 20);
            fgets(update, 20, check_up);
            printf("stm.bin version update : %s\n", update);
            strcpy(stm_upd, update);
            memset(update, 0, 20);
            fgets(update, 20, check_up);
            printf("esp.bin version update : %s\n", update);
            strcpy(esp_upd, update);
            memset(update, 0, 20);
            //printf("%s\n %s\n %s\n %s\n %s\n ", pay_upd, dbc_upd, cmd_upd, ecu_upd, stm_upd);

            if (pay_upd[0] == '1')
            {
              update_status |= 1 << 4;
            }
            if (dbc_upd[0] == '1')
            {
              update_status |= 1 << 3;
            }
            if (cmd_upd[0] == '1')
            {
              update_status |= 1 << 2;
            }
            if (ecu_upd[0] == '1')
            {
              update_status |= 1 << 1;
            }
            if (stm_upd[0] == '1')
            {
              update_status |= 1 << 0;
            }

            if (esp_upd[0] == '1')
            {
              update_status |= 1 << 0;
            }

            printf("update_status %x\n", update_status);
            sprintf(st_cmd, "$T9%02X", update_status);

            //PAYLOAD BIN UPDATE

            if (pay_upd[0] == '1')
            {
              printf("UPDATE IN PAYLOAD\n");

              uart_write_bytes(EX_UART_NUM, st_cmd, 10); //PAYLOAD BIN UPDATE

              tempstorage = fopen(MOUNT_POINT "/payload.bin", "wb");

              strcpy(download_filename, pay);

              if (!tempstorage)
              {
                fprintf(stderr, "Can't open temp storage payload.bin file\n");
              }

              IFT_start = 0;

              IFT_start = Server_Download(download_filename, tempstorage);

              if (IFT_start)
              {
                printf("Failure downloading Payload\n");
                uart_write_bytes(EX_UART_NUM, "Xa\n", 2);
              }
              else
              {
                printf("Downloaded Payload\n");
                uart_write_bytes(EX_UART_NUM, "$T205", 6);
              }
            }

            //dbc.csv UPDATE

            if (dbc_upd[0] == '1')
            {
              printf("UPDATE IN DBC\n");

              uart_write_bytes(EX_UART_NUM, st_cmd, 10); //dbc.csv UPDATE
              /***** dbc Download ********/

              tempstorage = fopen(MOUNT_POINT "/dbc.csv", "wb");

              strcpy(download_filename, DBC);

              if (!tempstorage)
              {
                fprintf(stderr, "Can't open temp storage dbc.csv file\n");
              }

              IFT_start = 0;

              IFT_start = Server_Download(download_filename, tempstorage);

              if (IFT_start)
              {
                printf("Failure downloading dbc\n");
                uart_write_bytes(EX_UART_NUM, "Xb\n", 2);
              }
              else
              {

                uart_write_bytes(EX_UART_NUM, "$T205", 6);
              }
            }

            //cmd.csv UPDATE

            if (cmd_upd[0] == '1')
            {
              printf("UPDATE IN CMD\n");

              uart_write_bytes(EX_UART_NUM, st_cmd, 10); //cmd.csv UPDATE

              /***** CMD Download ********/

              tempstorage = fopen(MOUNT_POINT "/cmd.csv", "wb");

              strcpy(download_filename, cmd);

              if (!tempstorage)
              {
                fprintf(stderr, "Can't open temp CMD.CSV storage file\n");
              }

              IFT_start = 0;

              IFT_start = Server_Download(download_filename, tempstorage);

              if (IFT_start)
              {
                printf("Failure downloading cmd\n");
                uart_write_bytes(EX_UART_NUM, "Xc\n", 2);
              }
              else
              {
                printf("Downloaded cmd\n");
                uart_write_bytes(EX_UART_NUM, "$T205", 6);
              }
            }

            //ECU BIN UPDATE

            if (ecu_upd[0] == '1')
            {
              printf("UPDATE IN ECU\n");

              uart_write_bytes(EX_UART_NUM, st_cmd, 10); //ECU BIN UPDATE

              /***** ECU Download ********/

              tempstorage = fopen(MOUNT_POINT "/ECU.bin", "wb");

              strcpy(download_filename, ECU);

              if (!tempstorage)
              {
                fprintf(stderr, "Can't open temp ECU.BIN storage file\n");
              }

              IFT_start = 0;

              IFT_start = Server_Download(download_filename, tempstorage);

              if (IFT_start)
              {
                printf("Failure downloading ECU\n");
                uart_write_bytes(EX_UART_NUM, "Xd\n", 2);
              }
              else
              {
                printf("Downloaded ECU\n");
                uart_write_bytes(EX_UART_NUM, "$T205", 6);
              }
            }

            //STM BIN UPDATE

            if (stm_upd[0] == '1')
            {
              printf("UPDATE IN STM\n");

              uart_write_bytes(EX_UART_NUM, st_cmd, 10); //STM BIN UPDATE

              /***** STM Download ********/

              tempstorage = fopen(MOUNT_POINT "/STM.bin", "wb");

              strcpy(download_filename, STM);

              if (!tempstorage)
              {
                fprintf(stderr, "Can't open temp storage file\n");
              }

              IFT_start = Server_Download(download_filename, tempstorage);

              if (IFT_start)
              {
                printf("Failure downloading STM\n");
                uart_write_bytes(EX_UART_NUM, "Xe\n", 2);
              }
              else
              {
                printf("Downloaded STM\n");
                uart_write_bytes(EX_UART_NUM, "$T205", 6);
              }
            }

            if (esp_upd[0] == '1')
            {
              printf("UPDATE IN ESP\n");

              uart_write_bytes(EX_UART_NUM, st_cmd, 10); //ECU BIN UPDATE

              /***** ECU Download ********/

              tempstorage = fopen(MOUNT_POINT "/ESP.bin", "wb");

              strcpy(download_filename, ESP);

              if (!tempstorage)
              {
                fprintf(stderr, "Can't open temp ESP.BIN storage file\n");
              }

              IFT_start = 0;

              IFT_start = Server_Download(download_filename, tempstorage);

              if (IFT_start)
              {
                printf("Failure downloading ESP\n");
                uart_write_bytes(EX_UART_NUM, "Xd\n", 2);
              }
              else
              {
                printf("Downloaded ESP\n");
                uart_write_bytes(EX_UART_NUM, "$T205", 6);
              }
            }

            /* IF THEIR IS NO UPDATE */

            if (pay_upd[0] == '0' && dbc_upd[0] == '0' && cmd_upd[0] == '0' && ecu_upd[0] == '0' && stm_upd[0] == '0')
            {
              uart_write_bytes(EX_UART_NUM, "$T20400", 8); //ALL FILES NEED UPDATE
            }

            /* CLOSE THE FILE */

            fclose(check_up);
          }

          else
          {
            // Wifi not connected Reattempting
            uart_write_bytes(EX_UART_NUM, "=\n", 1); //FAILURE
          }
        }

        /* STM ASKING ESP IF READY FOR IFT */
        else if ((data[1] == 'A') && (data[2] == '3'))
        {
          ESP_LOGI(AT, "$A3");
          ESP_LOGI(TAG, "inside file transfer");

          int rec_res = -1;
          char orig_name[256] = {'\0'};
          uint32_t max_fsize = 1024 * 1024 * 100;

          ESP_LOGI(TAG, "Opening file");
          FILE *f = fopen(MOUNT_POINT "/hello.csv", "w");
          if (f == NULL)
          {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
          }
          if (f)
          {
            //ESP READY FOR IFT

            uart_write_bytes(EX_UART_NUM, "$B301", 6); //AT COMMAND FOR ESP READY FOR IFT

            rec_res = Ymodem_Receive(f, max_fsize, orig_name);
          }
          fclose(f);
          ESP_LOGI(TAG, "File written");
          FILE *f_check = fopen(MOUNT_POINT "/hello.csv", "r");
          // Moving pointer to end
          fseek(f_check, 0, SEEK_END);
          long file_size_ift2 = ftell(f_check);
          printf("THE COMPLETED FILESIZE == %ld\n", file_size_ift2);
          fclose(f_check);
          // Printing position of pointer
          //printf("%ld", ftell(f_check));
          if (file_size_ift2 > ((file_size_ift / 100) * 95))
          {
            ESP_LOGI(TAG, "Renaming file");
            strcpy(filename_buffer_2, "/sdcard/");
            strncat(filename_buffer_2, orig_name, 12);
            if (rename(MOUNT_POINT "/hello.csv", filename_buffer_2) != 0)
            {
              ESP_LOGE(TAG, "Rename of hello.csv failed");
              return;
            }
            FILE *f_db = fopen(MOUNT_POINT "/filedb.csv", "a");
            fwrite(filename_buffer_2, 1, 20, f_db);
            fwrite("\n", 1, 1, f_db);
            fclose(f_db);
            FILE *f_lf = fopen(MOUNT_POINT "/LASTFILE.csv", "w");
            if (f_lf == NULL)
            {
              ESP_LOGE(TAG, "Failed to open file for reading");
              return;
            }
            fwrite(orig_name, 1, 12, f_lf);
            fwrite("\n", 1, 1, f_lf);
            fclose(f_lf);
          }
        }

        /* TRANSMIT THE NAME OF LAST FILE THAT WAS SENT TO ESP */
        else if (data[1] == 'A' && data[2] == '2')
        {
          ESP_LOGI(AT, "$A2");
          //NEW CODE
          //AT COMMAND FOR LAST FILE : - $B201T_000000X.CSV NEEDED TO BE ADDED
          char lastfilename[18] = "";
          char cmd[30] = "$B201";
          FILE *f_lf = fopen(MOUNT_POINT "/LASTFILE.csv", "r");
          if (f_lf == NULL)
          {
            ESP_LOGE(TAG, "Failed to open file for reading");
            char *defaultfn = "_000.CSV\n";
            uart_write_bytes(EX_UART_NUM, "$B201T_000000.CSV", 18);
            return;
          }
          ESP_LOGI(TAG, "LASTFILE REQUEST\n");
          fgets(lastfilename, sizeof(lastfilename), f_lf);
          printf("Lastfile:%s", lastfilename);
          fclose(f_lf);
          strncat(cmd, lastfilename, 18);
          uart_write_bytes(EX_UART_NUM, (char *)cmd, sizeof(cmd));
        }

        // else if (data[1] == 'B' && data[2] == 'D')
        // {
        //   char lastfilename[15] = "";
        //   FILE *f_lf = fopen(MOUNT_POINT "/LDF.csv", "r");
        //   if (f_lf == NULL)
        //   {
        //     ESP_LOGE(TAG, "Failed to open file for reading");
        //     uart_write_bytes(EX_UART_NUM, "20060400.CSV\n", 13);
        //     return;
        //   }
        //   ESP_LOGI(TAG, "LASTFILE REQUEST\n");
        //   fgets(lastfilename, sizeof(lastfilename), f_lf);
        //   printf("Lastfile:%s", lastfilename);
        //   fclose(f_lf);

        //   uart_write_bytes(EX_UART_NUM, (char *)lastfilename, sizeof(lastfilename));
        // }

        /****** All files downloaeded ******/

        else if (data[1] == 'T' && data[2] == '0')
        {
          ESP_LOGI(TAG, "inside file transfer");

          int rec_res = -1;
          char orig_name[256] = {'\0'};
          uint32_t max_fsize = 1024 * 1024 * 100;

          ESP_LOGI(TAG, "Opening file");
          FILE *f = fopen(MOUNT_POINT "/hello.csv", "w");
          if (f == NULL)
          {
            ESP_LOGE(TAG, "Failed to open file for writing");
            return;
          }
          if (f)
          {
            rec_res = Ymodem_Receive(f, max_fsize, orig_name);
          }
          fclose(f);
          ESP_LOGI(TAG, "File written");
          ESP_LOGI(TAG, "Renaming file");
          strcpy(filename_buffer_2, "/sdcard/");
          strncat(filename_buffer_2, orig_name, 10);
          if (rename(MOUNT_POINT "/hello.csv", filename_buffer_2) != 0)
          {
            ESP_LOGE(TAG, "Rename failed");
            return;
          }

          int a = Server_Upload();
          printf("Server Upload %d\n", a);
        }

        /* READY FOR IFT OF PAYLOAD.BIN */
        else if (data[1] == 'A' && data[2] == '4')
        {
          ESP_LOGI(AT, "$A4");
          uart_write_bytes(EX_UART_NUM, "$B401", 6); //AT COMMAND FOR IFT OF PAYLOAD.BIN

          printf("IFT Starts");

          /***** Payload *********/

          IFT_file = fopen(MOUNT_POINT "/payload.bin", "r");

          if (!IFT_file)
          {
            uart_write_bytes(EX_UART_NUM, "$B402", 6); //AT COMMAND FOR NOT READY FOR IFT OF PAYLOAD.BIN
            fprintf(stderr, "Can't open temp storage payload.bin file\n");
          }

          fseek(IFT_file, 0, SEEK_END);
          file_alavu = ftell(IFT_file);

          fclose(IFT_file);

          printf("Payload: %ld\n", file_alavu);

          IFT_file = fopen(MOUNT_POINT "/payload.bin", "r");

          result = Ymodem_Transmit("payload.bin", file_alavu, IFT_file);

          fclose(IFT_file);

          printf("Result Payload IFT: %d\n", result);

          vTaskDelay(2000 / portTICK_RATE_MS);

          if (result)
          {
            uart_write_bytes(EX_UART_NUM, "$B404", 6); //AT COMMAND FOR FAILURE OF IFT FROM ESP
            printf("Fail File 1\n");
          }
          else
          {
            uart_write_bytes(EX_UART_NUM, "$B403", 6); //AT COMMAND FOR SUCCESSFUL IFT FROM ESP
            printf("Success File 1\n");
          }
        }

        /* READY FOR IFT OF DBC.CSV */
        else if (data[1] == 'A' && data[2] == '5')
        {
          ESP_LOGI(AT, "$A5");

          uart_write_bytes(EX_UART_NUM, "$B501", 6); //AT COMMAND FOR IFT OF DBC.CSV

          /**** dbc ******/

          IFT_file = fopen(MOUNT_POINT "/dbc.csv", "r");

          if (!IFT_file)
          {
            uart_write_bytes(EX_UART_NUM, "$B502", 6); //AT COMMAND FOR NOT READY FOR IFT OF DBC.CSV
            fprintf(stderr, "Can't open temp storage file\n");
          }

          fseek(IFT_file, 0, SEEK_END);
          file_alavu = ftell(IFT_file);

          fclose(IFT_file);

          printf("DBC: %ld\n", file_alavu);

          IFT_file = fopen(MOUNT_POINT "/dbc.csv", "r");

          result = Ymodem_Transmit("dbc.csv", file_alavu, IFT_file);

          fclose(IFT_file);

          printf("Result DBC IFT: %d\n", result);

          vTaskDelay(2000 / portTICK_RATE_MS);

          if (result)
          {
            uart_write_bytes(EX_UART_NUM, "$B504", 6); //AT COMMAND FOR FAILURE OF IFT FROM ESP
            printf("Fail File 1\n");
          }
          else
          {
            uart_write_bytes(EX_UART_NUM, "$B503", 6); //AT COMMAND FOR SUCCESSFUL IFT FROM ESP
            printf("Success File 1\n");
          }
        }

        /* READY FOR IFT OF CMD.CSV */
        else if (data[1] == 'A' && data[2] == '6')
        {
          ESP_LOGI(AT, "$A6");

          uart_write_bytes(EX_UART_NUM, "$B601", 6); //AT COMMAND FOR IFT OF CMD.CSV

          /***** CMD *********/

          IFT_file = fopen(MOUNT_POINT "/cmd.csv", "r");

          if (!IFT_file)
          {
            uart_write_bytes(EX_UART_NUM, "$B602", 6); //AT COMMAND FOR NOT READY FOR IFT OF CMD.CSV
            fprintf(stderr, "Can't open temp storage file\n");
          }

          fseek(IFT_file, 0, SEEK_END);
          file_alavu = ftell(IFT_file);

          fclose(IFT_file);

          printf("CMD: %ld\n", file_alavu);

          IFT_file = fopen(MOUNT_POINT "/cmd.csv", "r");

          result = Ymodem_Transmit("cmd.csv", file_alavu, IFT_file);

          fclose(IFT_file);

          printf("Result CMD IFT: %d\n", result);

          vTaskDelay(2000 / portTICK_RATE_MS);

          if (result)
          {
            uart_write_bytes(EX_UART_NUM, "$B604", 6); //AT COMMAND FOR FAILURE OF IFT FROM ESP
            printf("Fail File 1\n");
          }
          else
          {
            uart_write_bytes(EX_UART_NUM, "$B603", 6); //AT COMMAND FOR SUCCESSFUL IFT FROM ESP
            printf("Success File 1\n");
          }
        }

        /* READY FOR IFT OF ECU.BIN */
        else if (data[1] == 'A' && data[2] == '7')
        {
          ESP_LOGI(AT, "$A7");

          uart_write_bytes(EX_UART_NUM, "$B701", 6); //AT COMMAND FOR IFT OF ECU.BIN

          /***** ECU *********/

          IFT_file = fopen(MOUNT_POINT "/ECU.bin", "r");

          if (!IFT_file)
          {
            uart_write_bytes(EX_UART_NUM, "$B702", 6); //AT COMMAND FOR NOT READY FOR IFT OF ECU.BIN
            fprintf(stderr, "Can't open temp storage file\n");
          }

          fseek(IFT_file, 0, SEEK_END);
          file_alavu = ftell(IFT_file);

          fclose(IFT_file);

          printf("ECU: %ld\n", file_alavu);

          IFT_file = fopen(MOUNT_POINT "/ECU.bin", "r");

          result = Ymodem_Transmit("ECU.bin", file_alavu, IFT_file);

          fclose(IFT_file);

          printf("Result ECU IFT: %d\n", result);

          vTaskDelay(2000 / portTICK_RATE_MS);

          if (result)
          {
            uart_write_bytes(EX_UART_NUM, "$B704", 6); //AT COMMAND FOR FAILURE OF IFT FROM ESP
            printf("Fail File 1\n");
          }
          else
          {
            uart_write_bytes(EX_UART_NUM, "$B703", 6); //AT COMMAND FOR SUCCESSFUL IFT FROM ESP
            printf("Success File 1\n");
          }
        }

        /* READY FOR IFT OF STM.BIN */
        else if (data[1] == 'A' && data[2] == '8')
        {
          ESP_LOGI(AT, "$A8");
          uart_write_bytes(EX_UART_NUM, "$B801", 6); //AT COMMAND FOR IFT OF STM.BIN
          /***** STM *********/

          IFT_file = fopen(MOUNT_POINT "/STM.bin", "r");

          if (!IFT_file)
          {
            uart_write_bytes(EX_UART_NUM, "$B802", 6); //AT COMMAND FOR NOT READY FOR IFT OF STM.BIN
            fprintf(stderr, "Can't open temp storage file\n");
          }

          fseek(IFT_file, 0, SEEK_END);
          file_alavu = ftell(IFT_file);

          fclose(IFT_file);

          printf("STM: %ld\n", file_alavu);

          IFT_file = fopen(MOUNT_POINT "/STM.bin", "r");

          result = Ymodem_Transmit("STM.bin", file_alavu, IFT_file);

          fclose(IFT_file);

          printf("Result STM IFT: %d\n", result);

          vTaskDelay(2000 / portTICK_RATE_MS);

          if (result)
          {
            uart_write_bytes(EX_UART_NUM, "$B804", 6); //AT COMMAND FOR FAILURE OF IFT FROM ESP
            printf("Fail File 1\n");
          }
          else
          {
            uart_write_bytes(EX_UART_NUM, "$B803", 6); //AT COMMAND FOR SUCCESSFUL IFT FROM ESP
            printf("Success File 1\n");
          }
        }

        else if (data[1] == 'M' && data[2] == '1')
        {
          while (1)
          {
            audio_playback_handler(keep_ign_on);
            vTaskDelay(25000 / portTICK_RATE_MS);
          }
        }
        else if (data[1] == 'M' && data[2] == '2')
        {
          audio_playback_handler(turn_vehicle_on);
        }
        else if (data[1] == 'M' && data[2] == '3')
        {
          audio_playback_handler(turn_vehicle_off);
        }
        else if (data[1] == 'M' && data[2] == '4')
        {
          audio_playback_handler(flashing_complete);
        }
        else if (data[1] == 'M' && data[2] == '5')
        {
          audio_playback_handler(press_button);
        }
        else if (data[1] == 'M' && data[2] == '6')
        {
          audio_playback_handler(update_available);
        }
        else if (data[1] == 'M' && data[2] == '7')
        {
          audio_playback_handler(backup_audio);
        }
        else if (data[1] == 'M' && data[2] == '8')
        {
          audio_playback_handler(PRESS_CLUTCH);
        }
        else if (data[1] == 'M' && data[2] == '9')
        {
          audio_playback_handler(RELEASE_CLUTCH);
        }
        else if (data[1] == 'M' && data[2] == 'A')
        {
          audio_playback_handler(PRESS_BRAKE);
        }
        else if (data[1] == 'M' && data[2] == 'B')
        {
          audio_playback_handler(RELEASE_BRAKE);
        }
        else if (data[1] == 'M' && data[2] == 'C')
        {
          audio_playback_handler(FIRST_GEAR);
        }
        else if (data[1] == 'M' && data[2] == 'D')
        {
          audio_playback_handler(NEUTRAL);
        }
        else if (data[1] == 'M' && data[2] == 'E')
        {
          audio_playback_handler(ENG_ON);
        }
        else if (data[1] == 'M' && data[2] == 'F')
        {
          audio_playback_handler(PRESS_THROTTLE);
        }
        else if (data[1] == 'M' && data[2] == 'G')
        {
          audio_playback_handler(RELEASE_THROTTLE);
        }
        else if (data[1] == 'M' && data[2] == 'H')
        {
          audio_playback_handler(ENG_OFF);
        }
        else if (data[1] == 'M' && data[2] == 'I')
        {
          audio_playback_handler(BRD_DONE);
        }
        else if (data[1] == 'M' && data[2] == 'J')
        {
          audio_playback_handler(REQ_DONE);
        }

        else if (data[1] == 'R' && data[2] == 'T')
        {
        }
      }
    }
  }
  free(data);
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

  //Set uart pattern detect function.
  //uart_enable_pattern_det_intr(EX_UART_NUM, '+', 3, 10000, 10, 10);
  xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);

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
 

  /*DEVICE READY TO ACCEPT AT COMMANDS*/
  uart_write_bytes(EX_UART_NUM, "@", 1);

  printf("CODE VERSION: %s\n", VERSION_NO);//VERSION OF CODE PRINTED

  printf("Started\n");


  while (1)
  {
    rx_task_2();
  }
}
