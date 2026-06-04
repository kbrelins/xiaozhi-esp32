#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <stdio.h>
#include "custom_lcd_display.h"
#include "wifi_board.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "codecs/box_audio_codec.h"
#include "wifi_station.h"
#include "mcp_server.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"

#define TAG "waveshare_rlcd_4_2"

// ═══════════════════════════════════════════════════════════════════
//  SHTC3 — stałe protokołu
// ═══════════════════════════════════════════════════════════════════
#define SHTC3_I2C_ADDR    0x70
#define SHTC3_MEASURE_MS  15      // czas pomiaru [ms]
#define SHTC3_INTERVAL_MS 60000   // odczyt co 30 s

// Stałe handle — rejestrujemy urządzenie raz i trzymamy przez cały czas.
// Dzięki temu nie ma konfliktu z ESP-IDF przy równoległych operacjach
// na magistrali przez ES8311/ES7210.
static i2c_master_dev_handle_t s_shtc3_dev = nullptr;

// ═══════════════════════════════════════════════════════════════════
//  Globalne dane sensora
// ═══════════════════════════════════════════════════════════════════
static float             s_temp      = 0.0f;
static float             s_hum       = 0.0f;
static float             s_temp_cal  = 10.0f; // Kalibracja czujnika temperatury
static float             s_hum_cal   = 10.0f; // Kalibracja czujnika temperatury
static bool              s_sensor_ok = false;
static SemaphoreHandle_t s_mutex     = nullptr;

// Wskaźniki na etykiety LVGL
static lv_obj_t* s_lbl_temp = nullptr;
static lv_obj_t* s_lbl_hum  = nullptr;

// ═══════════════════════════════════════════════════════════════════
//  CRC-8 (wielomian 0x31, init 0xFF)
// ═══════════════════════════════════════════════════════════════════
static uint8_t Shtc3Crc(const uint8_t* data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════════════
//  Jednorazowa rejestracja urządzenia + soft-reset czujnika.
//  Soft-reset (0x805D) wprowadza SHTC3 w znany stan (Sleep mode)
//  bez względu na to, co działo się wcześniej.
// ═══════════════════════════════════════════════════════════════════
static bool Shtc3RegisterDevice(i2c_master_bus_handle_t bus) {
    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address  = SHTC3_I2C_ADDR;
    cfg.scl_speed_hz    = 400000;
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &s_shtc3_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SHTC3: rejestracja I2C nieudana: %s", esp_err_to_name(err));
        return false;
    }
    // Soft-reset — wymuś znany stan czujnika przy starcie
    const uint8_t soft_reset[] = {0x80, 0x5D};
    i2c_master_transmit(s_shtc3_dev, soft_reset, sizeof(soft_reset), 20);
    vTaskDelay(pdMS_TO_TICKS(1));   // czujnik potrzebuje <1 ms na reset
    ESP_LOGI(TAG, "SHTC3: urządzenie zarejestrowane (0x%02X), soft-reset wykonany",
             SHTC3_I2C_ADDR);
    return true;
}

// ═══════════════════════════════════════════════════════════════════
//  Jeden pełny cykl odczytu SHTC3:
//  wakeup (1 ms) → measure cmd → 15 ms → read 6B → CRC
//
//  UWAGA: NIE wysyłamy komendy sleep na końcu.
//  Datasheet SHTC3 §3.1: czujnik automatycznie wraca do trybu Sleep
//  po ~1 s bezczynności. Jawna komenda sleep pozostawiała magistralę
//  I2C w błędnym stanie (ESP_ERR_INVALID_STATE co drugi cykl).
// ═══════════════════════════════════════════════════════════════════
static bool Shtc3Read(float& temp, float& hum) {
    if (!s_shtc3_dev) {
        ESP_LOGE(TAG, "SHTC3: brak zarejestrowanego urządzenia");
        return false;
    }

    // 1. Wybudź czujnik (ze stanu Sleep)
    const uint8_t wakeup[] = {0x35, 0x17};
    esp_err_t err = i2c_master_transmit(s_shtc3_dev, wakeup, sizeof(wakeup), 20);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SHTC3: błąd wakeup (%s)", esp_err_to_name(err));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));   // datasheet: min. 240 µs po wakeup

    // 2. Uruchom pomiar (T najpierw, tryb normalny, bez clock-stretching)
    const uint8_t measure[] = {0x7C, 0xA2};
    err = i2c_master_transmit(s_shtc3_dev, measure, sizeof(measure), 20);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SHTC3: błąd komendy pomiaru (%s)", esp_err_to_name(err));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(SHTC3_MEASURE_MS));

    // 3. Odczytaj 6 bajtów: [T_H, T_L, CRC_T, H_H, H_L, CRC_H]
    uint8_t data[6] = {};
    err = i2c_master_receive(s_shtc3_dev, data, sizeof(data), 50);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SHTC3: błąd odczytu danych (%s)", esp_err_to_name(err));
        return false;
    }

    // 4. Weryfikacja CRC
    if (Shtc3Crc(data, 2) != data[2]) {
        ESP_LOGW(TAG, "SHTC3: CRC temperatury niezgodne (got 0x%02X, expected 0x%02X)",
                 data[2], Shtc3Crc(data, 2));
        return false;
    }
    if (Shtc3Crc(data + 3, 2) != data[5]) {
        ESP_LOGW(TAG, "SHTC3: CRC wilgotności niezgodne (got 0x%02X, expected 0x%02X)",
                 data[5], Shtc3Crc(data + 3, 2));
        return false;
    }

    // 5. Przelicz wartości (wzory z datasheet)
    uint16_t raw_t = (uint16_t)((data[0] << 8) | data[1]);
    uint16_t raw_h = (uint16_t)((data[3] << 8) | data[4]);
    temp = (175.0f * (float)raw_t / 65536.0f - 45.0f)  - s_temp_cal; //-45.0f + 175.0f * (float)raw_t / 65536.0f;  // T = -45 + 175 * rawValue / 2^16
    hum  = (100.0f * (float)raw_h / 65536.0f) + s_hum_cal;  // RH = rawValue / 2^16 * 100

    // Bez jawnej komendy sleep — czujnik auto-śpi po ~1 s (datasheet §3.1)
    return true;
}

// ═══════════════════════════════════════════════════════════════════
//  Task FreeRTOS: cykliczny odczyt w tle
//  Stos 8 kB — potrzebne dla ESP_LOGI i operacji I2C + FreeRTOS.
// ═══════════════════════════════════════════════════════════════════
static void SensorTask(void* /*arg*/) {
    // Pierwsze odczytanie po 2 s — daj czas na boot audio codeca
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (true) {
        float t = 0.0f, h = 0.0f;
        bool  ok = Shtc3Read(t, h);

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (ok) {
            s_temp      = t;
            s_hum       = h;
            s_sensor_ok = true;
            ESP_LOGI(TAG, "SHTC3: T=%.1f°C  H=%.1f%%", t, h);
        } else {
            s_sensor_ok = false;
            ESP_LOGW(TAG, "SHTC3: odczyt nieudany, następna próba za %d s",
                     SHTC3_INTERVAL_MS / 1000);
        }
        xSemaphoreGive(s_mutex);

        vTaskDelay(pdMS_TO_TICKS(SHTC3_INTERVAL_MS));
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Timer LVGL: odświeża etykiety (wywołanie w wątku LVGL)
// ═══════════════════════════════════════════════════════════════════
static void SensorUiRefresh(lv_timer_t* /*timer*/) {
    if (!s_lbl_temp || !s_lbl_hum) return;

    float t, h;
    bool  ok;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    t  = s_temp;
    h  = s_hum;
    ok = s_sensor_ok;
    xSemaphoreGive(s_mutex);

    char buf[24];
    if (ok) {
        // \xc2\xb0 = znak stopnia ° w UTF-8
        // Jeśli wyświetla się '?' zamiast °, zmień na: "T: %.1f st.C"
        snprintf(buf, sizeof(buf), "T: %.1f\xc2\xb0""C", t);
        lv_label_set_text(s_lbl_temp, buf);
        snprintf(buf, sizeof(buf), "H: %.1f%%", h);
        lv_label_set_text(s_lbl_hum, buf);
    } else {
        lv_label_set_text(s_lbl_temp, "T: ---");
        lv_label_set_text(s_lbl_hum,  "H: ---");
    }
}

// ═══════════════════════════════════════════════════════════════════
//  Jednorazowy timer LVGL: tworzy widżety po SetupUI()
//
//  POZYCJONOWANIE:
//  Ekran ma 400×300 px. Typowy układ xiaozhi-esp32:
//    y=  0..36  → pasek statusu (ikony, sieć, bateria)
//    y= 36..~50 → ← tutaj umieszczamy dane czujnika
//    y= 50..260 → treść (emoji)
//    y=260..300 → etykieta tekstu/powiadomień
//
//  Jeśli chcesz przesunąć — zmień wartości SENSOR_Y_OFFSET poniżej.
// ═══════════════════════════════════════════════════════════════════
#define SENSOR_Y_OFFSET  38   // piksele od góry ekranu, pod paskiem statusu

static void CreateSensorWidgetOnce(lv_timer_t* timer) {
    lv_timer_delete(timer);   // usuń ten jednorazowy timer

    lv_obj_t* scr = lv_scr_act();

    // Temperatura — lewa strona, pod paskiem statusu
    s_lbl_temp = lv_label_create(scr);
    lv_obj_set_pos(s_lbl_temp, 45, SENSOR_Y_OFFSET); // pozycja X zmieniona z 8 na 45
    lv_label_set_text(s_lbl_temp, "T: ...");

    // Wilgotność — prawa strona, ta sama linia
    s_lbl_hum = lv_label_create(scr);
    lv_obj_set_pos(s_lbl_hum, 220, SENSOR_Y_OFFSET);
    lv_label_set_text(s_lbl_hum, "H: ...");

    ESP_LOGI(TAG, "LVGL: widżety czujnika utworzone (y=%d)", SENSOR_Y_OFFSET);

    // Cykliczny timer odświeżania wartości
    lv_timer_create(SensorUiRefresh, SHTC3_INTERVAL_MS, nullptr);

    // Pierwsze odświeżenie od razu (dane mogą już być dostępne)
    SensorUiRefresh(nullptr);
}

// ═══════════════════════════════════════════════════════════════════
//  Klasa płytki
// ═══════════════════════════════════════════════════════════════════
class CustomBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t   i2c_bus_;
    Button                    boot_button_;
    CustomLcdDisplay*         display_;
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t         cali_handle;
    bool                      vbat_status = false;

    void InitializeI2c() {
        i2c_master_bus_config_t cfg = {};
        cfg.i2c_port             = ESP32_I2C_HOST;
        cfg.sda_io_num           = AUDIO_CODEC_I2C_SDA_PIN;  // GPIO13
        cfg.scl_io_num           = AUDIO_CODEC_I2C_SCL_PIN;  // GPIO14
        cfg.clk_source           = I2C_CLK_SRC_DEFAULT;
        cfg.glitch_ignore_cnt    = 7;
        cfg.intr_priority        = 0;
        cfg.trans_queue_depth    = 0;
        cfg.flags.enable_internal_pullup = 1;
        ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &i2c_bus_));
    }

    void InitializeShtc3() {
        s_mutex = xSemaphoreCreateMutex();
        configASSERT(s_mutex);

        // Zarejestruj urządzenie SHTC3 raz — trzymamy handle przez cały czas
        Shtc3RegisterDevice(i2c_bus_);

        // Task odczytu: stos 8 kB (margines na I2C + logging)
        xTaskCreate(SensorTask, "shtc3_task", 8192, nullptr, 1, nullptr);

        // Zaplanuj tworzenie widżetów LVGL za 10 s
        // (LVGL jest gotowy od InitializeLcdDisplay, ale SetupUI() wywołuje
        //  dopiero Application — stąd opóźnienie)
        lvgl_port_lock(0);
        lv_timer_create(CreateSensorWidgetOnce, 10000, nullptr);
        lvgl_port_unlock();
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(
                    app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif
    }

    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        mcp.AddTool("self.disp.network", "Ponowna konfiguracja sieci Wi-Fi",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                EnterWifiConfigMode();
                return true;
            });

        mcp.AddTool("self.sensor.temperature",
            "Odczytaj temperaturę z wbudowanego czujnika SHTC3 [°C]",
            PropertyList(),
            [](const PropertyList&) -> ReturnValue {
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                float t  = s_temp;
                bool  ok = s_sensor_ok;
                xSemaphoreGive(s_mutex);
                if (!ok) return std::string("Czujnik niedostępny");
                char buf[32];
                snprintf(buf, sizeof(buf), "%.1f°C", t);
                return std::string(buf);
            });

        mcp.AddTool("self.sensor.humidity",
            "Odczytaj wilgotność względną z wbudowanego czujnika SHTC3 [%]",
            PropertyList(),
            [](const PropertyList&) -> ReturnValue {
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                float h  = s_hum;
                bool  ok = s_sensor_ok;
                xSemaphoreGive(s_mutex);
                if (!ok) return std::string("Czujnik niedostępny");
                char buf[32];
                snprintf(buf, sizeof(buf), "%.1f%%", h);
                return std::string(buf);
            });
    }

    void InitializeLcdDisplay() {
        spi_display_config_t spi = {};
        spi.mosi = RLCD_MOSI_PIN;
        spi.scl  = RLCD_SCK_PIN;
        spi.dc   = RLCD_DC_PIN;
        spi.cs   = RLCD_CS_PIN;
        spi.rst  = RLCD_RST_PIN;
        display_ = new CustomLcdDisplay(
            nullptr, nullptr,
            RLCD_WIDTH, RLCD_HEIGHT,
            DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
            DISPLAY_SWAP_XY,
            spi);
    }

    uint16_t BatterygetVoltage() {
        static bool                      initialized = false;
        static adc_oneshot_unit_handle_t adc_handle;
        static adc_cali_handle_t         cali        = nullptr;

        if (!initialized) {
            adc_oneshot_unit_init_cfg_t init_cfg = {.unit_id = ADC_UNIT_1};
            adc_oneshot_new_unit(&init_cfg, &adc_handle);
            adc_oneshot_chan_cfg_t ch_cfg = {
                .atten    = ADC_ATTEN_DB_12,
                .bitwidth = ADC_BITWIDTH_12,
            };
            adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_3, &ch_cfg);
            adc_cali_curve_fitting_config_t cali_cfg = {
                .unit_id  = ADC_UNIT_1,
                .atten    = ADC_ATTEN_DB_12,
                .bitwidth = ADC_BITWIDTH_12,
            };
            if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali) == ESP_OK)
                initialized = true;
        }

        if (initialized) {
            int raw = 0, mv = 0;
            adc_oneshot_read(adc_handle, ADC_CHANNEL_3, &raw);
            adc_cali_raw_to_voltage(cali, raw, &mv);
            return (uint16_t)(mv * 3);
        }
        return 0;
    }

    uint8_t BatterygetPercent() {
        int v = 0;
        for (int i = 0; i < 10; i++) v += BatterygetVoltage();
        v /= 10;
        int pct = (-1 * v * v + 9016 * v - 19189000) / 10000;
        pct = (pct > 100) ? 100 : (pct < 0) ? 0 : pct;
        return (uint8_t)pct;
    }

public:
    CustomBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeLcdDisplay();   // inicjuje LVGL — musi być przed SHTC3
        InitializeShtc3();        // rejestruje I2C, startuje task, planuje timer LVGL
        InitializeButtons();
        InitializeTools();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec codec(
            i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging,
                                 bool& discharging) override {
        charging    = false;
        discharging = !charging;
        level       = (int)BatterygetPercent();
        return true;
    }
};

DECLARE_BOARD(CustomBoard);
