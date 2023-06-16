#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include "ds18b20.h"
#include "esp_timer.h"
#include "ssd1306.h"
#include <string.h>

#define TRIGGER_PIN GPIO_NUM_13 // pino trigger do sensor ultrassonico
#define ECHO_PIN GPIO_NUM_35    // pino echo do sensor ultrassonico
#define WATER_TANK_HEIGHT_CM 20 // altura maxima do reservatorio
#define PIN_DS18B20 GPIO_NUM_32 // pino data do sensor de temp
#define DISTANCE_CONTROL GPIO_NUM_10
#define TEMPERATURE_CONTROL GPIO_NUM_9
#define DECREASE_BUTTON GPIO_NUM_14
#define INCREMENT_BUTTON GPIO_NUM_26
#define CHANGE_MODE_BUTTON GPIO_NUM_27
#define DEBOUNCE_MS 200
#define READ_SENSORS_DELAY 2000

#define DS18B20_TAG "DS18B20"
#define HCSR04_TAG "HCSR04"
#define DECREASE_BUTTON_TAG "DECREASE"
#define INCREMENT_BUTTON_TAG "INCREMENT"
#define CHANGE_MODE_BUTTON_TAG "CHANGE_MODE"

#define TEMPERATURE_MODE 1
#define DISTANCE_MODE 2

#define delay(value) vTaskDelay(value / portTICK_PERIOD_MS)

volatile double temperatureLimit = 10;
volatile int storageCapacityLimit = 10;
volatile double waterDistance = 0;
volatile float waterTemperature = 0;

TickType_t last_tick_decrease = 0;
volatile bool decrease_button = false;

TickType_t last_tick_increment = 0;
volatile bool increment_button = false;

TickType_t last_tick_change_mode = 0;
volatile bool change_mode_button = false;

static SSD1306_t dev;

volatile int currentMode = DISTANCE_MODE;

double calculateWaterPercent()
{
    return ((WATER_TANK_HEIGHT_CM - waterDistance) / WATER_TANK_HEIGHT_CM) * 100;
}

void write_text()
{
    char strTemperature[12];
    char strDistance[12];
    char strTemperatureLimit[12];
    char strDistanceLimit[12];

    float waterPercentage = calculateWaterPercent();

    // Mostrar no display valores atuais de temperatura e capacidade
    sprintf(strDistance, "%.2f", waterPercentage);
    sprintf(strTemperature, "%.2f", waterTemperature);
    strcat(strDistance, " %");
    strcat(strTemperature, " C");
    ssd1306_display_text(&dev, 0, "Niveis atuais", 14, false);
    ssd1306_display_text(&dev, 1, strDistance, strlen(strDistance), false);
    ssd1306_display_text(&dev, 2, strTemperature, strlen(strTemperature), false);

    // Mostrar no display valores limites para acionamento dos atuadores
    sprintf(strDistanceLimit, "%d", storageCapacityLimit);
    sprintf(strTemperatureLimit, "%.2f", temperatureLimit);
    if (currentMode == DISTANCE_MODE)
    {
        strcat(strDistanceLimit, " % <-");
        strcat(strTemperatureLimit, " C");
    }
    else
    {
        strcat(strDistanceLimit, " %");
        strcat(strTemperatureLimit, " C <-");
    }

    ssd1306_display_text(&dev, 4, "Configuracoes", 14, false);
    ssd1306_display_text(&dev, 5, strDistanceLimit, strlen(strDistanceLimit), false);
    ssd1306_display_text(&dev, 6, strTemperatureLimit, strlen(strTemperatureLimit), false);
}

void hcsr04_task(void *pvParameters)
{
    esp_rom_gpio_pad_select_gpio(TRIGGER_PIN);
    esp_rom_gpio_pad_select_gpio(ECHO_PIN);
    esp_rom_gpio_pad_select_gpio(DISTANCE_CONTROL);
    gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(DISTANCE_CONTROL, GPIO_MODE_OUTPUT);

    while (1)
    {
        // Pulso no pino de trigger
        gpio_set_level(TRIGGER_PIN, 1);
        delay(10);
        gpio_set_level(TRIGGER_PIN, 0);

        // Aguardar resposta no pino echo
        int64_t pulse_start = 0;
        int64_t pulse_end = 0;

        while (gpio_get_level(ECHO_PIN) == 0)
        {
            pulse_start = esp_timer_get_time();
        }
        while (gpio_get_level(ECHO_PIN) == 1)
        {
            pulse_end = esp_timer_get_time();
        }

        // Calcular a duração do pulso em microssegundos
        int64_t pulse_duration = pulse_end - pulse_start;

        // Calcular a distância em centímetros
        waterDistance = pulse_duration * 0.0343 / 2; // Fórmula para calcular a distância
        float waterPercentage = calculateWaterPercent();

        if (waterPercentage < storageCapacityLimit)
        {
            ESP_LOGW(HCSR04_TAG, "Bomba acionada!");
            gpio_set_level(DISTANCE_CONTROL, 0);
        }
        else
        {
            gpio_set_level(DISTANCE_CONTROL, 1);
        }

        ssd1306_clear_line(&dev, 1, false);
        write_text();

        ESP_LOGW(HCSR04_TAG, "Distancia: %.2f cm", waterDistance);
        ESP_LOGW(HCSR04_TAG, "Porcentagem de agua: %.2f %%\n", waterPercentage);
        delay(READ_SENSORS_DELAY);
    }
}

void temperature_task(void *pvParameters)
{
    ds18b20_init(PIN_DS18B20);
    esp_rom_gpio_pad_select_gpio(TEMPERATURE_CONTROL);
    gpio_set_direction(TEMPERATURE_CONTROL, GPIO_MODE_OUTPUT);
    while (1)
    {
        float current_temp = ds18b20_get_temp();
        if (current_temp < temperatureLimit)
        {
            ESP_LOGE(DS18B20_TAG, "Resistência acionada");
            gpio_set_level(TEMPERATURE_CONTROL, 0);
        }
        else
        {
            gpio_set_level(TEMPERATURE_CONTROL, 1);
        }

        ssd1306_clear_line(&dev, 2, false);
        write_text();

        ESP_LOGE(DS18B20_TAG, "Temperature: %0.2f C\n", current_temp);
        waterTemperature = current_temp;
        delay(READ_SENSORS_DELAY);
    }
}

void IRAM_ATTR isrKeyDecrease(void *arg)
{
    TickType_t now_tick = xTaskGetTickCountFromISR();
    if (((now_tick - last_tick_decrease) >= pdMS_TO_TICKS(DEBOUNCE_MS)) && !decrease_button)
    {
        last_tick_decrease = now_tick;
        decrease_button = true;
    }
}

void IRAM_ATTR isrKeyIncrement(void *arg)
{
    TickType_t now_tick = xTaskGetTickCountFromISR();
    if (((now_tick - last_tick_increment) >= pdMS_TO_TICKS(DEBOUNCE_MS)) && !increment_button)
    {
        last_tick_increment = now_tick;
        increment_button = true;
    }
}

void IRAM_ATTR isrKeyChangeMode(void *arg)
{
    TickType_t now_tick = xTaskGetTickCountFromISR();
    if (((now_tick - last_tick_change_mode) >= pdMS_TO_TICKS(DEBOUNCE_MS)) && !change_mode_button)
    {
        last_tick_change_mode = now_tick;
        change_mode_button = true;
    }
}

void setup_buttons()
{
    gpio_config_t in_conf = {};
    in_conf.intr_type = GPIO_INTR_NEGEDGE;
    in_conf.mode = GPIO_MODE_INPUT;
    in_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    in_conf.pull_up_en = GPIO_PULLUP_ENABLE;

    in_conf.pin_bit_mask = (1 << DECREASE_BUTTON);
    gpio_config(&in_conf);

    in_conf.pin_bit_mask = (1 << INCREMENT_BUTTON);
    gpio_config(&in_conf);

    in_conf.pin_bit_mask = (1 << CHANGE_MODE_BUTTON);
    gpio_config(&in_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(DECREASE_BUTTON, isrKeyDecrease, NULL);
    gpio_isr_handler_add(INCREMENT_BUTTON, isrKeyIncrement, NULL);
    gpio_isr_handler_add(CHANGE_MODE_BUTTON, isrKeyChangeMode, NULL);
}

void decrease_button_task(void *pvParams)
{
    for (;;)
    {
        if (decrease_button)
        {
            decrease_button = false;
            if (currentMode == TEMPERATURE_MODE)
            {
                if (temperatureLimit > 10)
                {
                    temperatureLimit--;
                    ssd1306_clear_line(&dev, 6, false);
                }
            }
            else
            {
                if (storageCapacityLimit > 10)
                {
                    storageCapacityLimit -= 5;
                    ssd1306_clear_line(&dev, 5, false);
                }
            }
            write_text();
            ESP_LOGI(DECREASE_BUTTON_TAG, "Diminuir valor\n");
        }
        delay(50);
    }
}

void increment_button_task(void *pvParams)
{
    for (;;)
    {
        if (increment_button)
        {
            increment_button = false;
            if (currentMode == TEMPERATURE_MODE)
            {
                if (temperatureLimit < 50)
                {
                    temperatureLimit++;
                    ssd1306_clear_line(&dev, 6, false);
                }
            }
            else
            {
                if (storageCapacityLimit < 100)
                {
                    storageCapacityLimit += 5;
                    ssd1306_clear_line(&dev, 5, false);
                }
            }
            write_text();
            ESP_LOGI(INCREMENT_BUTTON_TAG, "Aumentar valor\n");
        }
        delay(50);
    }
}

void change_mode_button_task(void *pvParams)
{
    for (;;)
    {
        if (change_mode_button)
        {
            if (currentMode == TEMPERATURE_MODE)
            {
                currentMode = DISTANCE_MODE;
            }
            else
            {
                currentMode = TEMPERATURE_MODE;
            }
            ssd1306_clear_line(&dev, 5, false);
            ssd1306_clear_line(&dev, 6, false);
            write_text();
            change_mode_button = false;
            ESP_LOGI(CHANGE_MODE_BUTTON_TAG, "Mudar modo: %d\n", currentMode);
        }
        delay(50);
    }
}

void app_main()
{
    uint32_t usStackDepth = 1024;
    setup_display_text(&dev);
    setup_buttons();

    xTaskCreatePinnedToCore(&hcsr04_task, "hcsr04_task", usStackDepth * 2, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(&temperature_task, "temperature_task", usStackDepth * 2, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(&decrease_button_task, "decrease_button_task", usStackDepth * 2, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(&increment_button_task, "increment_button_task", usStackDepth * 2, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(&change_mode_button_task, "change_mode_button_task", usStackDepth * 2, NULL, 1, NULL, 0);
}
