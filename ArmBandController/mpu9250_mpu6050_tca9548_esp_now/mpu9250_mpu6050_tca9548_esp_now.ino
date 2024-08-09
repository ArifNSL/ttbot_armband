#include <Wire.h>
#include "MPU9250.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "TCA9548.h"
#include <esp_now.h>
#include <WiFi.h>

// Connection indicator led
// Blink = Data delivery is unsuccessful (responder not found yet)
// LED on = Data delivery is successful (sending message to responder)
const int indicator_led = 2;

// MAC Address of responder esp32
uint8_t broadcastAddress[] = {0xE8, 0x6B, 0xEA, 0xD0, 0xC8, 0x70};

// Peer info
esp_now_peer_info_t peerInfo;

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

// Create an instance of the TCA9548
TCA9548 multiplexer(0x70); // Initialize TCA9548A with address 0x70

// Initialize MPU9250 for channel 2
MPU9250 mpu_channel_2;

// MPU6050 instances
MPU6050 mpu_6050_1;
MPU6050 mpu_6050_2;

#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady_6050_1 = false;
bool dmpReady_6050_2 = false;
uint8_t mpuIntStatus_6050_1;
uint8_t mpuIntStatus_6050_2;
uint8_t devStatus_6050_1;
uint8_t devStatus_6050_2;
uint16_t packetSize_6050_1;
uint16_t packetSize_6050_2;
uint8_t fifoBuffer_6050_1[64];
uint8_t fifoBuffer_6050_2[64];

// orientation/motion vars
Quaternion q_6050_1;
VectorFloat gravity_6050_1;
float ypr_6050_1[3];

Quaternion q_6050_2;
VectorFloat gravity_6050_2;
float ypr_6050_2[3];

// Variables to store initial orientation for MPU9250
float initialYaw = 0.0;
float initialPitch = 0.0;
float initialRoll = 0.0;

// Homing button variables
const int button_pin = 15;
bool button_pressed = false;
float elbow[3];
float elbow_offset[3];
float wrist[3];
float wrist_offset[3];

// Flex sensor variables
const int flex_sensor_pin = 34;   // ADC pin where the potentiometer is connected
const int initialSamples = 10;    // Number of samples for the initial averaging
const int smoothingSamples = 5;   // Number of samples for further smoothing
const int readInterval = 5;       // Interval between readings in milliseconds


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Increase I2C clock speed to 400 kHz
  
  pinMode(button_pin, INPUT_PULLUP);
  pinMode(indicator_led, OUTPUT);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Initialize multiplexer
  if (!multiplexer.begin()) {
    Serial.println("TCA9548A not found!");
    while (1);
  }


  // Initialize and calibrate MPU6050 on channels 0 and 1
  initialize_mpu6050(&mpu_6050_1, 0);
  initialize_mpu6050(&mpu_6050_2, 1);

  // Initialize and calibrate MPU9250 on channel 2
  calibrate_and_setup_mpu(&mpu_channel_2, 2);

  // Record initial orientation for MPU9250
  record_initial_orientation(&mpu_channel_2, 2);
}


/**************************************************************************/
/*
    Arduino loop function, 
*/
/**************************************************************************/
void loop() {
  handleButton();

  // Flex sensor reading and smoothing
  static unsigned long previousMillis = 0;
  static int initialSampleCount = 0;
  static long initialSum = 0;
  static int smoothingSampleCount = 0;
  static long smoothingSum = 0;
  static int finalSmoothedValue = 0;

  unsigned long currentMillis = millis();

  if (initialSampleCount < initialSamples) {
    if (currentMillis - previousMillis >= readInterval) {
      previousMillis = currentMillis;
      initialSum += analogRead(flex_sensor_pin);
      initialSampleCount++;
    }
  } else if (smoothingSampleCount < smoothingSamples) {
    if (currentMillis - previousMillis >= readInterval) {
      previousMillis = currentMillis;
      int initialAverage = initialSum / initialSamples;
      int mappedValue = map(initialAverage, 1970, 2700, 0, 200); // 0 is closed gripper
      smoothingSum += mappedValue;
      smoothingSampleCount++;
    }
  } else {
    finalSmoothedValue = smoothingSum / smoothingSamples;

    // Constrain the final smoothed value to be within the range suitable for robotic arm's gripper
     finalSmoothedValue = constrain(finalSmoothedValue, 0, 100);

    // Print the final smoothed mapped value
    Serial.print("Final Smoothed Mapped Value: ");
    Serial.println(finalSmoothedValue);

    // Reset counters and sums for the next reading cycle
    initialSampleCount = 0;
    initialSum = 0;
    smoothingSampleCount = 0;
    smoothingSum = 0;

    // Add claw value to message
    message.claw = finalSmoothedValue;

    // uncomment the necessary one and comment out the other one
//    message.id = 1; // Right arm
     message.id = 2; // Left arm

    // Delay between complete reading cycles
    delay(50);
  }

  // Read data from MPU9250 and MPU6050s
  multiplexer.selectChannel(0);
  read_and_send_data_6050(&mpu_6050_1, &q_6050_1, &gravity_6050_1, ypr_6050_1, fifoBuffer_6050_1, packetSize_6050_1, &mpuIntStatus_6050_1, "6050_1");

  multiplexer.selectChannel(1);
  read_and_send_data_6050(&mpu_6050_2, &q_6050_2, &gravity_6050_2, ypr_6050_2, fifoBuffer_6050_2, packetSize_6050_2, &mpuIntStatus_6050_2, "6050_2");

  multiplexer.selectChannel(2);
  read_and_send_data_9250(2);
  
  // Send the message with all data
  send_message(&message, sizeof(message));
}


/**************************************************************************/
/*
    Initiate and calibrate MPU9250
*/
/**************************************************************************/
void calibrate_and_setup_mpu(MPU9250* mpu, int channel) {
  multiplexer.selectChannel(channel);
  delay(50); // Delay to allow the multiplexer to settle
  
  if (!mpu->setup(0x68)) {  // MPU9250 address is 0x68
    while (1) {
      Serial.print("MPU connection failed on channel ");
      Serial.print(channel);
      Serial.println(". Please check your connection.");
      delay(5000);
    }
  }

  // Calibrate accelerometer and gyroscope
  Serial.print("Accel Gyro calibration for channel ");
  Serial.print(channel);
  Serial.println(" will start in 2sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu->verbose(true);
  delay(2000);
  mpu->calibrateAccelGyro();

  // Calibrate magnetometer
  Serial.print("Mag calibration for channel ");
  Serial.print(channel);
  Serial.println(" will start in 2sec.");
  Serial.println("Please wave the device in a figure eight until done.");
  delay(2000);
  mpu->calibrateMag();
}


/**************************************************************************/
/*
    Initiate and calibrate MPU6050
*/
/**************************************************************************/
void initialize_mpu6050(MPU6050* mpu, int channel) {
    multiplexer.selectChannel(channel);
    Serial.print(F("Initializing MPU6050 on channel "));
    Serial.print(channel);
    Serial.println(F("..."));
    mpu->initialize();
    Serial.print(F("Testing MPU6050 connection on channel "));
    Serial.print(channel);
    Serial.println(F("..."));
    Serial.println(mpu->testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.print(F("Initializing DMP on channel "));
    Serial.print(channel);
    Serial.println(F("..."));
    uint8_t devStatus = mpu->dmpInitialize();
    mpu->setXGyroOffset(220);
    mpu->setYGyroOffset(76);
    mpu->setZGyroOffset(-85);
    mpu->setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu->CalibrateAccel(6);
    mpu->CalibrateGyro(6);
    mpu->PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu->setDMPEnabled(true);

    if (channel == 0) {
      mpuIntStatus_6050_1 = mpu->getIntStatus();
      dmpReady_6050_1 = true;
      packetSize_6050_1 = mpu->dmpGetFIFOPacketSize();
    } else if (channel == 1) {
      mpuIntStatus_6050_2 = mpu->getIntStatus();
      dmpReady_6050_2 = true;
      packetSize_6050_2 = mpu->dmpGetFIFOPacketSize();
    }
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

    multiplexer.disableChannel(channel);
}


/**************************************************************************/
/*
    Assign MPU9250 readings to the message
*/
/**************************************************************************/
void read_and_send_data_9250(int channel) {
  MPU9250* mpu = &mpu_channel_2;

  // Read MPU9250 data
  if (mpu->update()) {
    int yaw = mpu->getYaw() - initialYaw;
    int pitch = mpu->getPitch() - initialPitch;
    int roll = mpu->getRoll() - initialRoll;

    // Convert yaw, pitch, and roll angles to the specified ranges
    if (yaw < 0) {
      yaw += 360;
    }
    // if (pitch < 0) {
    //   pitch += 180;
    // }
    // if (roll < 0) {
    //   roll += 360;
    // }

    // Set data in the message
    message.yaw_9250 = yaw;
    message.pitch_9250 = pitch;
    message.roll_9250 = roll;

    // Print angles to Serial monitor
    Serial.print("MPU9250 - Yaw: ");
    Serial.print(message.yaw_9250);
    Serial.print("\tPitch: ");
    Serial.print(message.pitch_9250);
    Serial.print("\tRoll: ");
    Serial.println(message.roll_9250);
  }
}


/**************************************************************************/
/*
    Assign MPU6050 readings to the message
*/
/**************************************************************************/
void read_and_send_data_6050(MPU6050* mpu, Quaternion* q, VectorFloat* gravity, float* ypr, uint8_t* fifoBuffer, uint16_t packetSize, uint8_t* mpuIntStatus, const char* mpu_name) {
  if (!dmpReady_6050_1) return;
  *mpuIntStatus = mpu->getIntStatus();
  uint16_t fifoCount = mpu->getFIFOCount();
  if ((*mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu->resetFIFO();
    Serial.println("FIFO overflow!");
  } else if (*mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu->getFIFOCount();
    mpu->getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu->dmpGetQuaternion(q, fifoBuffer);
    mpu->dmpGetGravity(gravity, q);
    mpu->dmpGetYawPitchRoll(ypr, q, gravity);
    Serial.print(mpu_name);
    Serial.print(" - Yaw: ");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print(", Pitch: ");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print(", Roll: ");
    Serial.println(ypr[2] * 180/M_PI);

    if (mpu_name == "6050_1") {
        // Sends original orientation until button is pressed
        if (button_pressed == false) {
          wrist[0] = ypr[0];
          wrist[1] = ypr[1];
          wrist[2] = ypr[2];
          }
        // Sets the offset value for 0 home whenever button is pressed
        // Debouncing is not necessary here
        if (digitalRead(button_pin) == LOW) {
          wrist_offset[0] = ypr[0];
          wrist_offset[1] = ypr[1];
          wrist_offset[2] = ypr[2];
          }
        // Keeps sending updated orientation once the button is pressed
        if (button_pressed == true) {
          wrist[0] = ypr[0] - wrist_offset[0];
          wrist[1] = ypr[1] - wrist_offset[1];
          wrist[2] = ypr[2] - wrist_offset[2];
          }
        // Register orientations in the message
        message.yaw_6050_1 = wrist[0] * 180 / M_PI;
        message.pitch_6050_1 = wrist[1] * 180 / M_PI;
        message.roll_6050_1 = wrist[2] * 180 / M_PI;
      }
      // Forearm sensor
      else {
        if (button_pressed == false) {
          elbow[0] = ypr[0];
          elbow[1] = ypr[1];
          elbow[2] = ypr[2];
          }
        // Keeps sending updated orientation once the button is pressed
        // Debouncing is not necessary here
        if (digitalRead(button_pin) == LOW) { 
          elbow_offset[0] = ypr[0];
          elbow_offset[1] = ypr[1];
          elbow_offset[2] = ypr[2];
       }
        // Keeps sending updated orientation once the button is pressed
        if (button_pressed == true) {
          elbow[0] = ypr[0] - elbow_offset[0];
          elbow[1] = ypr[1] - elbow_offset[1];
          elbow[2] = ypr[2] - elbow_offset[2];
        }
        // Register orientations in the message
        message.yaw_6050_2 = elbow[0] * 180 / M_PI;
        message.pitch_6050_2 = elbow[1] * 180 / M_PI;
        message.roll_6050_2 = elbow[2] * 180 / M_PI;
    }
  }
}


/**************************************************************************/
/*
    Record initial orientation for MPU9250
*/
/**************************************************************************/
void record_initial_orientation(MPU9250* mpu, int channel) {
  initialYaw = mpu->getYaw();
  initialPitch = mpu->getPitch();
  initialRoll = mpu->getRoll();
}


/**************************************************************************/
/*
    Send message using ESP-NOW
*/
/**************************************************************************/
void send_message(MessageData *message, size_t size) {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)message, size);
  
  if (result == ESP_OK) {
    Serial.print("Sent with success: ");
//    Serial.print(message->yaw_6050_1);
//    Serial.print("\t");
//    Serial.print(message->pitch_6050_1);
//    Serial.print("\t");
//    Serial.print(message->roll_6050_1);
//    Serial.print("\t");
//    Serial.print(message->yaw_6050_2);
//    Serial.print("\t");
//    Serial.print(message->pitch_6050_2);
//    Serial.print("\t");
//    Serial.print(message->roll_6050_2);
//    Serial.print("\t");
//    Serial.print(message->yaw_9250);
//    Serial.print("\t");
//    Serial.print(message->pitch_9250);
//    Serial.print("\t");
//    Serial.print(message->roll_9250);
//    Serial.print("\t");
//    Serial.print(message->claw);
//    Serial.print("\t");
//    Serial.println(message->id);
  } else {
    Serial.println("Error sending the message");
  }
}


/**************************************************************************/
/*
    Handle the buttonpress with debouncing
*/
/**************************************************************************/
void handleButton() {
  static bool buttonState = false;
  static bool lastButtonState = false;
  static unsigned long lastDebounceTime = 0;
  static unsigned long debounceDelay = 30;

  unsigned long currentMillis = millis();

  // Button debouncing
  bool reading = digitalRead(button_pin);
  if (reading != lastButtonState) {
    lastDebounceTime = currentMillis;
  }

  if ((currentMillis - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // Toggle action
      if (buttonState) {
        button_pressed = true;
        Serial.println("Button pressed!");
      }
    }
  }

  lastButtonState = reading;
}
