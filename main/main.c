#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"
#include "driver/i2s_types.h"

#include "driver/gpio.h"
#include "esp_log.h"

// 引脚参考：
// https://wiki.seeedstudio.com/cn/xiao_esp32c6_getting_started/#%E7%A1%AC%E4%BB%B6%E6%A6%82%E8%A7%88

// 扬声器的引脚

/**
 * 
为什么换用不同的 GPIO 引脚后有声音了
这是一个很好的观察！原因与 ESP32C6 的引脚特性和功能有关：

为什么 GPIO 0/1/2 无声音
特殊引脚功能冲突：

GPIO_NUM_0：是启动模式选择引脚（按下 BOOT 按钮时会拉低）
GPIO_NUM_1/2：通常被保留用于 UART0 (TX/RX)，用于日志输出和固件下载
系统预留：

ESP32C6 启动时，这些引脚可能被系统临时控制，导致信号不稳定
驱动能力限制：

部分引脚可能内部驱动能力不同，不适合 I2S 的时序要求
为什么 GPIO 17/23/16 有声音
专用功能支持：

GPIO_NUM_17 (RX)：有 I2S 专用功能，适合作为 WS 信号
GPIO_NUM_23 (SCL)：适合作为时钟线使用，提供稳定的时钟信号
GPIO_NUM_16 (TX)：适合数据传输，有发送功能支持
无系统冲突：

这些引脚在启动后不会被系统复用，更为专一
电气特性：

这些引脚可能有更适合数字音频传输的电气特性
设计建议
在 ESP32 系列芯片中，选择引脚时应遵循：

尽量避免使用特殊引脚（GPIO 0/1/2/3）
使用芯片厂商推荐的功能专用引脚
查阅开发板引脚图上的标记（如 SCL、RX、TX 等）作为选择指南
换用 17/23/16 是正确的选择，因为它们是 ESP32C6 上更适合 I2S 功能的引脚。

ESP32C6:
// 扬声器的引脚
#define MAX_LRC GPIO_NUM_17 // D7 (RX)
#define MAX_BCLK GPIO_NUM_23 // D5 (SCL)
#define MAX_DIN GPIO_NUM_16 // D6 (TX)
 */

/*
M5Stack ATOM S3R
#define MAX_LRC GPIO_NUM_7
#define MAX_BCLK GPIO_NUM_6
#define MAX_DIN GPIO_NUM_5

#define INMP_SD GPIO_NUM_8
#define INMP_SCK GPIO_NUM_6
#define INMP_WS GPIO_NUM_7
*/

#define MAX_LRC GPIO_NUM_16
#define MAX_BCLK GPIO_NUM_15
#define MAX_DIN GPIO_NUM_7

// 麦克风的引脚
#define INMP_SD GPIO_NUM_6
#define INMP_SCK GPIO_NUM_5 // MAX_BCLK = INMP_SCK
#define INMP_WS GPIO_NUM_4 // MAX_LRC = INMP_WS

// 配置扬声器/麦克风的采样率
#define SAMPLE_RATE 44100

// i2s_common: dma frame num is out of dma buffer size, limited to 1023
#define DMA_FRAME_NUM 1023
#define SLOT_NUM I2S_SLOT_MODE_MONO // 单声道
#define SLOT_BIT_WIDTH I2S_DATA_BIT_WIDTH_16BIT // 16bit

// https://docs.espressif.com/projects/esp-idf/zh_CN/stable/esp32/api-reference/peripherals/i2s.html#id13
// 这里的一帧是指一个 WS 周期内的所有采样数据。因此， dma_buffer_size = dma_frame_num * slot_num * slot_bit_width / 8
#define DMA_BUF_SIZE (DMA_FRAME_NUM * SLOT_NUM * SLOT_BIT_WIDTH / 8)

// 音频buffer
uint8_t use_buffer[DMA_BUF_SIZE] = {0};

i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;

extern const uint8_t music_pcm_start[] asm("_binary_canon_pcm_start");
extern const uint8_t music_pcm_end[]   asm("_binary_canon_pcm_end");

// MAX98357A 扬声器 写数据 
static void i2s_tx_init() {
    // 用 I2S1 作为主机（Master）初始化一个 I2S 通道，用于音频数据的发送（播放）。
    i2s_chan_config_t tx_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    tx_config.dma_frame_num = DMA_FRAME_NUM;
    
    // ⚠️
    ESP_ERROR_CHECK(i2s_new_channel(&tx_config, &tx_handle, NULL));

    i2s_std_config_t tx_std_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, SLOT_NUM),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .din = I2S_GPIO_UNUSED,
            .bclk = MAX_BCLK,
            .ws = MAX_LRC,
            // ⚠️
            .dout = MAX_DIN,
            .invert_flags = {
                .bclk_inv = false,
                .ws_inv = false,
                .mclk_inv = false,
            }
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &tx_std_config));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
}

// INMP441 麦克风 读数据
static void i2s_rx_init() {
    // i2s_chan_config_t rx_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    i2s_chan_config_t rx_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    rx_config.dma_frame_num = DMA_FRAME_NUM;

    // ⚠️
    ESP_ERROR_CHECK(i2s_new_channel(&rx_config, NULL, &rx_handle));
    // ESP_ERROR_CHECK(i2s_new_channel(&rx_config, &tx_handle, &rx_handle));

    i2s_std_config_t rx_std_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, SLOT_NUM),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            // ⚠️
            .din = INMP_SD,
            .bclk = INMP_SCK,
            .ws = INMP_WS,
            .dout = I2S_GPIO_UNUSED,
            .invert_flags = {
                .bclk_inv = false,
                .ws_inv = false,
                .mclk_inv = false,
            }
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &rx_std_config));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

static void mic_2_speaker(void *arg)
{
    size_t bytes = 0;
    char *TAG = "mic_2_speaker";
    
    ESP_LOGI(TAG, "开始麦克风到扬声器传输");
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        esp_err_t read_ret = i2s_channel_read(rx_handle, use_buffer, DMA_BUF_SIZE, &bytes, 1000);
        
        if(read_ret == ESP_OK && bytes > 0) {
            
            esp_err_t write_ret = i2s_channel_write(tx_handle, use_buffer, DMA_BUF_SIZE, &bytes, 1000);
            if (write_ret != ESP_OK) {
                ESP_LOGE(TAG, "写入扬声器失败: %s", esp_err_to_name(write_ret));
            }
        } else {
            ESP_LOGI(TAG, "读取麦克风失败: %s, 字节数: %d", esp_err_to_name(read_ret), bytes);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void play_music(void *arg)
{
    char *TAG = "play_music";
    esp_err_t ret = ESP_OK;
    size_t bytes_write = 0;
    uint8_t *data_ptr = (uint8_t *)music_pcm_start;

    /* (Optional) Disable TX channel and preload the data before enabling the TX channel,
     * so that the valid data can be transmitted immediately */
    ESP_ERROR_CHECK(i2s_channel_disable(tx_handle));
    ESP_ERROR_CHECK(i2s_channel_preload_data(tx_handle, data_ptr, music_pcm_end - data_ptr, &bytes_write));
    data_ptr += bytes_write;  // Move forward the data pointer

    /* Enable the TX channel */
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    while (1) {
        /* Write music to earphone */
        ret = i2s_channel_write(tx_handle, data_ptr, music_pcm_end - data_ptr, &bytes_write, portMAX_DELAY);
        if (ret != ESP_OK) {
            /* Since we set timeout to 'portMAX_DELAY' in 'i2s_channel_write'
               so you won't reach here unless you set other timeout value,
               if timeout detected, it means write operation failed. */
            ESP_LOGE(TAG, "[music] i2s write failed, %d", ret);
            abort();
        }
        if (bytes_write > 0) {
            ESP_LOGI(TAG, "[music] i2s music played, %d bytes are written.", bytes_write);
        } else {
            ESP_LOGE(TAG, "[music] i2s music play failed.");
            abort();
        }
        data_ptr = (uint8_t *)music_pcm_start;
        // 加了会造成卡顿
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void mic_test(void *arg)
{
    char *TAG = "mic_test";
    size_t bytes = 0;
    esp_err_t ret = ESP_OK;

    while (1) {
        ret = i2s_channel_read(rx_handle, use_buffer, DMA_BUF_SIZE, &bytes, 1000);
        if (ret == ESP_OK && bytes > 0) {
            ESP_LOGI(TAG, "读取麦克风成功: %d 字节", bytes);
        } else {
            ESP_LOGE(TAG, "读取麦克风失败: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// 以双工方式初始化I2S通道
void i2s_init() {
    // 使用单个I2S控制器
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    
    // 同时创建TX和RX句柄
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));

    i2s_std_slot_config_t speaker_std_slot_config = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(SLOT_BIT_WIDTH, SLOT_NUM);
    speaker_std_slot_config.bit_shift = true; // 使能位移
    
    // 扬声器配置 (TX)
    i2s_std_config_t tx_std_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = speaker_std_slot_config,
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .din = I2S_GPIO_UNUSED,
            .bclk = MAX_BCLK,
            .ws = MAX_LRC,
            // ⚠️
            .dout = MAX_DIN,
            .invert_flags = {
                .bclk_inv = false,
                .ws_inv = false,
                .mclk_inv = false,
            }
        },
    };
    
    // 麦克风配置 (RX)，使用共享时钟信号
    i2s_std_slot_config_t mic_std_slot_config = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(SLOT_BIT_WIDTH, SLOT_NUM);

    mic_std_slot_config.bit_shift = true; // 使能位移

    i2s_std_config_t rx_std_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = mic_std_slot_config,
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .dout = I2S_GPIO_UNUSED,
            .bclk = INMP_SCK,
            .ws = INMP_WS,
            // ⚠️
            .din = INMP_SD,
            .invert_flags = {
                .bclk_inv = false,
                .ws_inv = false,
                .mclk_inv = false,
            }
        },
    };
    
    // 初始化并启用TX和RX通道
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &tx_std_config));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &rx_std_config));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

void app_main(void)
{
    ESP_LOGI("app_main", "进入程序");
    // // 音频输出（播放）
    // i2s_tx_init();
    // ESP_LOGI("app_main", "初始化扬声器完毕");
    // // 音频输入（录音）
    // i2s_rx_init();
    // ESP_LOGI("app_main", "初始化麦克风完毕");
    // // xTaskCreate(mic_test, "mic_test", 4096, NULL, 5, NULL);
    i2s_init();
    ESP_LOGI("app_main", "初始化扬声器/麦克风完毕");
    xTaskCreate(mic_2_speaker, "mic_2_speaker", 4096 * 2, NULL, 5, NULL);
    // xTaskCreate(play_music, "play_music", 2048, NULL, 5, NULL);

}