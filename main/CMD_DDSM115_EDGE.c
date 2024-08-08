#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "cJSON.h" // Include the cJSON library

#define RS485_CONTROL_PIN GPIO_NUM_8
#define UART_PORT_NUM UART_NUM_1
#define BUF_SIZE (1024)
#define EX_UART_NUM UART_NUM_0


#define CURRENT_LOOP_MODE 0x01
#define SPEED_LOOP_MODE   0x02
#define POSITION_LOOP_MODE 0x03


static const char *TAG = "RS485";

static QueueHandle_t uart1_queue;

// CRC8 Calculation Function
uint8_t calculateCRC8(uint8_t* data, uint8_t length) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t extract = data[i];
        for (uint8_t tempI = 8; tempI; tempI--) {
            uint8_t sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum) {
                crc ^= 0x8C;
            }
            extract >>= 1;
        }
    }
    return crc;
}

// Transmission Control Functions
void preTransmission() {
    gpio_set_level(RS485_CONTROL_PIN, 1);
}

void postTransmission() {
    gpio_set_level(RS485_CONTROL_PIN, 0);
}

// Send Command Function
void sendCommand(uint8_t *command, size_t length) {
    preTransmission();
    uart_write_bytes(UART_PORT_NUM, (const char*)command, length);
    uart_wait_tx_done(UART_PORT_NUM, portMAX_DELAY);
    postTransmission();
}

// Conversion Functions
int16_t convertCurrentToData(float current) {
    return (int16_t)(current * 4095.875); // 32767 / 8 = 4095.875
}

float convertDataToCurrent(int16_t data) {
    return (float)data / 4095.875;
}

int16_t convertSpeedToData(float speed) {
    return (int16_t)speed;
}

float convertDataToSpeed(int16_t data) {
    return (float)data;
}

uint16_t convertPositionToData(float position) {
    return (uint16_t)(position * 91.0194); // 32767 / 360 = 91.0194
}

float convertDataToPosition(uint16_t data) {
    return (float)data / 91.0194;
}

// Command Functions
void switchToVelocityLoop(uint8_t mode) {
    uint8_t command[] = {0x01, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, mode};
    sendCommand(command, sizeof(command)); // No CRC calculation needed
}

void queryMotorMode() {
    uint8_t command[] = {0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
    sendCommand(command, sizeof(command));
}

void brakeCommand() {
    uint8_t command[] = {0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xD1};
    sendCommand(command, sizeof(command));
}

void setMotorID(uint8_t newID) {
    ESP_LOGI(TAG, "Setting motor ID to: %d", newID);

    // Prepare command
    uint8_t command[] = {0xAA, 0x55, 0x53, newID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Send command
    sendCommand(command, sizeof(command));

    // Log command sent
    ESP_LOGI(TAG, "Sent setMotorID command: ");
    for (int i = 0; i < sizeof(command); i++) {
        ESP_LOGI(TAG, "%02X", command[i]);
    }
}

void queryID() {
    uint8_t command[] = {0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE};
    sendCommand(command, sizeof(command));
}

void currentLoopCommand(uint8_t motorID, float current) {
    int16_t currentData = convertCurrentToData(current);
    uint8_t command[10] = {motorID, 0x64, (uint8_t)(currentData >> 8), (uint8_t)currentData, 0x00, 0x00, 0x00, 0x00, 0x00};
    command[9] = calculateCRC8(command, 9);
    ESP_LOGI(TAG, "Sending current command: ");
    for (int i = 0; i < sizeof(command); i++) {
        ESP_LOGI(TAG, "%02X", command[i]);
    }
    sendCommand(command, sizeof(command));
}

void velocityLoopCommand(uint8_t motorID, float speed) {
    int16_t speedData = convertSpeedToData(speed);
    uint8_t command[10] = {motorID, 0x64, (uint8_t)(speedData >> 8), (uint8_t)speedData, 0x00, 0x00, 0x00, 0x00, 0x00};
    command[9] = calculateCRC8(command, 9);
    ESP_LOGI(TAG, "Sending velocity command: ");
    for (int i = 0; i < sizeof(command); i++) {
        ESP_LOGI(TAG, "%02X", command[i]);
    }
    sendCommand(command, sizeof(command));
}

void positionLoopCommand(uint8_t motorID, float position) {
    uint16_t positionData = convertPositionToData(position);
    uint8_t command[10] = {motorID, 0x64, (uint8_t)(positionData >> 8), (uint8_t)positionData, 0x00, 0x00, 0x00, 0x00, 0x00};
    command[9] = calculateCRC8(command, 9);
    ESP_LOGI(TAG, "Sending position command: ");
    for (int i = 0; i < sizeof(command); i++) {
        ESP_LOGI(TAG, "%02X", command[i]);
    }
    sendCommand(command, sizeof(command));
}

void handleJSONCommand(const char *jsonString) {
    cJSON *json = cJSON_Parse(jsonString);
    if (json == NULL) {
        ESP_LOGE(TAG, "Invalid JSON string");
        return;
    }

    // Check for "BRAKE" command
    cJSON *brakeItem = cJSON_GetObjectItem(json, "BRAKE");
    if (brakeItem != NULL) {
        ESP_LOGI(TAG, "BRAKE command received");
        brakeCommand();
    }

    // Check for "QUERY MOTOR" command
    cJSON *queryMotorItem = cJSON_GetObjectItem(json, "QUERY MOTOR");
    if (queryMotorItem != NULL) {
        ESP_LOGI(TAG, "QUERY MOTOR command received");
        // queryMotorMode();
        // Assume you'll handle the response in the UART event task
    }

    // Check for "QUERY ID" command
    cJSON *queryIDItem = cJSON_GetObjectItem(json, "QUERY ID");
    if (queryIDItem != NULL) {
        ESP_LOGI(TAG, "QUERY ID command received");
        queryID();
    }

    // Handle other fields
    cJSON *idItem = cJSON_GetObjectItem(json, "ID");
    if (idItem != NULL) {
        uint8_t motorID = (uint8_t)idItem->valueint;
        ESP_LOGI(TAG, "Motor ID: %d", motorID);

        cJSON *positionItem = cJSON_GetObjectItem(json, "position");
        if (positionItem != NULL) {
            float position = (float)positionItem->valuedouble;
            positionLoopCommand(motorID, position);
        }

        cJSON *currentItem = cJSON_GetObjectItem(json, "current");
        if (currentItem != NULL) {
            float current = (float)currentItem->valuedouble;
            currentLoopCommand(motorID, current);
        }

        cJSON *velocityItem = cJSON_GetObjectItem(json, "velocity");
        if (velocityItem != NULL) {
            float velocity = (float)velocityItem->valuedouble;
            velocityLoopCommand(motorID, velocity);
        }

        cJSON *modeItem = cJSON_GetObjectItem(json, "mode");
        if (modeItem != NULL) {
            uint8_t mode = (uint8_t)modeItem->valueint;
            switchToVelocityLoop(mode);
        }
    }

    cJSON *newIDItem = cJSON_GetObjectItem(json, "newID");
    if (newIDItem != NULL) {
        uint8_t newID = (uint8_t)newIDItem->valueint;
        ESP_LOGI(TAG, "Setting new motor ID to: %d", newID);
        setMotorID(newID);
    }

    cJSON_Delete(json);
}

void decodeQueryMotorResponse(uint8_t *data) {
    if (data[0] != 0x01) {
        ESP_LOGW(TAG, "Unexpected ID in QUERY MOTOR response");
        return;
    }

    uint8_t mode = data[1];

    int16_t torqueCurrent;
    int16_t velocity;
    uint16_t position;

    // Decode based on mode
    switch (mode) {
        case CURRENT_LOOP_MODE:  // Replace with actual mode value for current loop
            torqueCurrent = (data[2] << 8) | data[3];
            float torqueCurrentValue = (torqueCurrent / 32767.0) * 8.0; // Convert to Amperes
            ESP_LOGI(TAG, "Torque Current: %f A", torqueCurrentValue);
            break;

        case SPEED_LOOP_MODE:  // Replace with actual mode value for speed loop
            velocity = (data[4] << 8) | data[5];
            float velocityValue = velocity; // Already in rpm
            ESP_LOGI(TAG, "Velocity: %f rpm", velocityValue);
            break;

        case POSITION_LOOP_MODE:  // Replace with actual mode value for position loop
            position = (data[6] << 8) | data[7];
            float positionValue = (position / 32767.0) * 360.0; // Convert to degrees
            ESP_LOGI(TAG, "Position: %f degrees", positionValue);
            break;

        default:
            ESP_LOGW(TAG, "Unknown mode value: %d", mode);
            return;
    }

    uint8_t errorCode = data[8];
    uint8_t crc8 = data[9]; // CRC8 should be validated

    // Validate CRC8
    uint8_t calculatedCRC8 = calculateCRC8(data, 9);
    if (calculatedCRC8 != crc8) {
        ESP_LOGW(TAG, "CRC8 mismatch: calculated %02X, received %02X", calculatedCRC8, crc8);
        return;
    }

    ESP_LOGI(TAG, "Decoded QUERY MOTOR Response:");
    ESP_LOGI(TAG, "Mode: %d", mode);
    // Logging for torqueCurrent, velocity, and position is handled inside the switch-case based on the mode.
    ESP_LOGI(TAG, "Error Code: %d", errorCode);
}



// UART Event Task
void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t data[BUF_SIZE];
    while (1) {
        if (xQueueReceive(uart1_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    if (event.size) {
                        int len = uart_read_bytes(UART_PORT_NUM, data, event.size, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "Received data event: ");
                        for (int i = 0; i < len; i++) {
                            ESP_LOGI(TAG, "%02X", data[i]);
                        }
                        decodeQueryMotorResponse(data); // Decode response if applicable
                    }
                    break;
                default:
                    break;
            }
        }
    }
}

// Main Application
void app_main(void) {
    // Configure UART1 (RS485)
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    if (uart_param_config(UART_PORT_NUM, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART");
        return;
    }
    if (uart_set_pin(UART_PORT_NUM, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins");
        return;
    }
    if (uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart1_queue, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver");
        return;
    }

    // Configure RS485 control pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RS485_CONTROL_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    postTransmission(); // Set initial state to listening

    // Configure UART0 (serial monitor)
    if (uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART0 driver");
        return;
    }
    if (uart_param_config(EX_UART_NUM, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART0");
        return;
    }
    if (uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART0 pins");
        return;
    }

    // Create UART event task
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 10, NULL);

    // Main loop to read from serial monitor and send over RS485
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return;
    }
    while (1) {
        // Read user input from the serial monitor
        int len = uart_read_bytes(EX_UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // Null-terminate the input string
            ESP_LOGI(TAG, "Read from UART0: %s", data);

            // Handle JSON command
            handleJSONCommand((const char*)data);

            // Log after handling command
            ESP_LOGI(TAG, "Handled command");
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay before next iteration
    }

    free(data);
}
