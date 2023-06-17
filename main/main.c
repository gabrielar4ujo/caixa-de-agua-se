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
#include "ultrasonic.h"

#define TRIGGER_PIN GPIO_NUM_13   // pino trigger do sensor ultrassonico
#define ECHO_PIN GPIO_NUM_35      // pino echo do sensor ultrassonico
#define WATER_TANK_HEIGHT_CM 13.5 // altura maxima do reservatorio
#define PIN_DS18B20 GPIO_NUM_32   // pino data do sensor de temp
#define DISTANCE_CONTROL GPIO_NUM_10
#define TEMPERATURE_CONTROL GPIO_NUM_9
#define DECREASE_BUTTON GPIO_NUM_14
#define INCREMENT_BUTTON GPIO_NUM_26
#define CHANGE_MODE_BUTTON GPIO_NUM_27
#define DEBOUNCE_MS 200
#define READ_SENSORS_DELAY 1500

#define DS18B20_TAG "DS18B20"
#define HCSR04_TAG "HCSR04"
#define DECREASE_BUTTON_TAG "DECREASE"
#define INCREMENT_BUTTON_TAG "INCREMENT"
#define CHANGE_MODE_BUTTON_TAG "CHANGE_MODE"

#define TEMPERATURE_MODE 1
#define DISTANCE_MODE 2

#define delay(value) vTaskDelay(value / portTICK_PERIOD_MS)

volatile double temperatureLimit = 10;
volatile int storageCapacityLimit = 100;
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

int calculateWaterPercent()
{
    float waterDistanceAdjust = 0.0;
    if (waterDistance < 3.5)
    {
        waterDistanceAdjust = 25.0;
    }
    int result = (((WATER_TANK_HEIGHT_CM - (waterDistance)) / WATER_TANK_HEIGHT_CM) * 100) + waterDistanceAdjust;

    if (result > 100)
        return 100;
    if (result < 0)
        return 0;
    return result;
}

void write_text()
{
    char strTemperature[12];
    char strDistance[12];
    char strTemperatureLimit[12];
    char strDistanceLimit[12];

    int waterPercentage = calculateWaterPercent();

    // Mostrar no display valores atuais de temperatura e capacidade
    sprintf(strDistance, "%d", waterPercentage);
    sprintf(strTemperature, "%0.1f", waterTemperature);
    strcat(strDistance, " %");
    strcat(strTemperature, " .C");
    ssd1306_display_text(&dev, 0, "Niveis atuais", 14, false);
    ssd1306_display_text(&dev, 1, strDistance, strlen(strDistance), false);
    ssd1306_display_text(&dev, 2, strTemperature, strlen(strTemperature), false);

    // Mostrar no display valores limites para acionamento dos atuadores
    sprintf(strDistanceLimit, "%d", storageCapacityLimit);
    sprintf(strTemperatureLimit, "%0.1f", temperatureLimit);
    if (currentMode == DISTANCE_MODE)
    {
        strcat(strDistanceLimit, " % <-");
        strcat(strTemperatureLimit, " .C");
    }
    else
    {
        strcat(strDistanceLimit, " %");
        strcat(strTemperatureLimit, " .C <-");
    }

    ssd1306_display_text(&dev, 4, "Configuracoes", 14, false);
    ssd1306_display_text(&dev, 5, strDistanceLimit, strlen(strDistanceLimit), false);
    ssd1306_display_text(&dev, 6, strTemperatureLimit, strlen(strTemperatureLimit), false);
}

void hcsr04_task(void *pvParameters)
{
    esp_rom_gpio_pad_select_gpio(DISTANCE_CONTROL);
    gpio_set_direction(DISTANCE_CONTROL, GPIO_MODE_OUTPUT);

    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_PIN,
        .echo_pin = ECHO_PIN,
    };
    ultrasonic_init(&sensor);

    while (1)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, WATER_TANK_HEIGHT_CM, &distance);
        if (res != ESP_OK)
        {
            ESP_LOGW(HCSR04_TAG, "Error %d: ", res);
            switch (res)
            {
            case ESP_ERR_ULTRASONIC_PING:
                ESP_LOGW(HCSR04_TAG, "Cannot ping (device is in invalid state)");
                break;
            case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                ESP_LOGW(HCSR04_TAG, "Ping timeout (no device found)");
                break;
            case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                ESP_LOGW(HCSR04_TAG, "Echo timeout (i.e. distance too big)");
                break;
            default:
                ESP_LOGW(HCSR04_TAG, "%s", esp_err_to_name(res));
            }
        }
        else
        {
            if (distance * 100 > WATER_TANK_HEIGHT_CM + 10)
            {
                ESP_LOGW(HCSR04_TAG, "Valor inválido: %f cm", distance * 100);
                gpio_set_level(DISTANCE_CONTROL, 1);
            }
            else
            {
                waterDistance = distance * 100;
                int waterPercentage = calculateWaterPercent();

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

                ESP_LOGW(HCSR04_TAG, "Distancia: %.1f cm", waterDistance);
                ESP_LOGW(HCSR04_TAG, "Porcentagem de agua: %d %%\n", waterPercentage);
            }
        }
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
        if (current_temp < -10 || current_temp > 50)
        {
            ESP_LOGE(DS18B20_TAG, "Valor inválido: %0.1f", current_temp);
        }
        else
        {
            if (current_temp < temperatureLimit)
            {
                ESP_LOGE(DS18B20_TAG, "Resistência acionada");
                gpio_set_level(TEMPERATURE_CONTROL, 1);
            }
            else
            {
                gpio_set_level(TEMPERATURE_CONTROL, 0);
            }

            ssd1306_clear_line(&dev, 2, false);
            write_text();

            ESP_LOGE(DS18B20_TAG, "Temperature: %0.1f C\n", current_temp);
            waterTemperature = current_temp;
            delay(READ_SENSORS_DELAY);
        }
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
    setup_display_text(&dev);
    setup_buttons();

    xTaskCreate(hcsr04_task, "hcsr04_task", configMINIMAL_STACK_SIZE * 4, NULL, 3, NULL);
    xTaskCreate(temperature_task, "temperature_task", configMINIMAL_STACK_SIZE * 4, NULL, 2, NULL);
    xTaskCreate(decrease_button_task, "decrease_button_task", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(increment_button_task, "increment_button_task", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    xTaskCreate(change_mode_button_task, "change_mode_button_task", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
}
