#include "MPU9250.h"
#include <esp_now.h>
#include <WiFi.h>

// Connection indicator led
// Blink = Data delivery is unsuccessful (responder not found yet)
// LED on = Data delivery is successful (sending message to responder)
const int indicator_led = 2;

MPU9250 mpu;

const uint8_t receiverMac[] = {0xE8, 0x6B, 0xEA, 0xD0, 0xC8, 0x70};

// Struct for the message data
typedef struct {
  int yaw_6050_1;
  int pitch_6050_1;
  int roll_6050_1;
  int yaw_6050_2;
  int pitch_6050_2;
  int roll_6050_2;
  int yaw_9250;
  int pitch_9250;
  int roll_9250;
  int claw;
  uint8_t id;
} MessageData;

MessageData message;

// Callback function called when data is sent
// LED indicates connection status
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Delivery Success");
    digitalWrite(indicator_led, HIGH);
  }
  else {
    Serial.println("Delivery Fail");
    digitalWrite(indicator_led, HIGH);
    delay(200);
    digitalWrite(indicator_led, LOW);
    delay(200);
  }
}


void setup() {
    Serial.begin(115200);
    Wire.begin();
    WiFi.mode(WIFI_STA);

    pinMode(indicator_led, OUTPUT);
    
    // Initialize ESP-NOW
    Serial.println("ESP-NOW Initializing...");
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register the send callback
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    delay(1000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // Calibrate the MPU
    Serial.println("Accel Gyro calibration will start in 3sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(3000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 3sec.");
    Serial.println("Please wave device in a figure eight until done.");
    delay(3000);
    mpu.calibrateMag();

    print_calibration();
    mpu.verbose(false);

    // Initialize message
    message.id = 3;
    message.yaw_6050_1 = 0;
    message.pitch_6050_1 = 0;
    message.roll_6050_1 = 0;
    message.yaw_6050_2 = 0;
    message.pitch_6050_2 = 0;
    message.roll_6050_2 = 0;
    message.yaw_9250 = 0;
    message.pitch_9250 = 0;
    message.roll_9250 = 0;
    message.claw = 0;
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            updateMessage();
            sendMessage();
            prev_ms = millis();
        }
    }
}

void updateMessage() {
    int yaw = static_cast<int>(mpu.getYaw());
//    if (yaw < 0) {
//      yaw += 360;
//  }
    message.yaw_9250 = yaw;
    message.pitch_9250 = static_cast<int>(mpu.getPitch());
    message.roll_9250 = static_cast<int>(mpu.getRoll());
}

void sendMessage() {
    esp_err_t result = esp_now_send(receiverMac, (uint8_t *) &message, sizeof(message));
    if (result == ESP_OK) {
        Serial.println("Message sent successfully");
    } else {
        Serial.println("Error sending the message");
    }
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
