#include <esp_now.h>
#include <WiFi.h>

// Define a data structure for the message
// Match it to the transmitted message structure for convenience
typedef struct struct_message {
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
} struct_message;

// Create a structured object
struct_message myData;

// Global variables
int yaw1 = 0, pitch1 = 0, roll1 = 0;
int yaw2 = 0, pitch2 = 0, roll2 = 0;
int yaw3 = 0, pitch3 = 0, roll3 = 0;
int yaw4 = 0, pitch4 = 0, roll4 = 0;
int yaw5 = 0, pitch5 = 0, roll5 = 0;
int yaw6 = 0, pitch6 = 0, roll6 = 0;
int yaw7 = 0, pitch7 = 0, roll7 = 0;
int yaw8 = 0, pitch8 = 0, roll8 = 0;
int yaw9 = 0, pitch9 = 0, roll9 = 0;
int newYaw9 = 0;
int claw1 = 100, claw2 = 100;

const int led_indicator = 2;       // Built in LED
unsigned long lastReceiveTime = 0; // Last time data was received
const unsigned long timeout = 5000; // Timeout interval in milliseconds


/**************************************************************************/
/*
    Callback function executed when data is received
*/
/**************************************************************************/
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  // id 1 --> right arm
  if (myData.id == 1) {
    yaw1 = myData.yaw_6050_1;
    pitch1 = myData.pitch_6050_1;
    roll1 = myData.roll_6050_1;

    if (myData.yaw_6050_2 > 180) {
      yaw2 = 360-myData.yaw_6050_2;
    }
    if (myData.yaw_6050_2 < -180) {
      yaw2 = -(myData.yaw_6050_2+360);
    }
    
    pitch2 = myData.pitch_6050_2;
    roll2 = myData.roll_6050_2;

    yaw3 = myData.yaw_9250-newYaw9;
    if (yaw3 < 0) {
      yaw3 += 360;
    } else if (yaw3 >= 360) {
      yaw3 -= 360;
    }
    pitch3 = myData.pitch_9250;
    roll3 = myData.roll_9250;

    claw1 = myData.claw;

    Serial.print(yaw1);
    Serial.print("\t");
    Serial.print(pitch1);
    Serial.print("\t");
    Serial.print(roll1);
    Serial.print("\t");
  
    Serial.print(yaw2);
    Serial.print("\t");
    Serial.print(pitch2);
    Serial.print("\t");
    Serial.print(roll2);
    Serial.print("\t");
  
    Serial.print(yaw3);
    Serial.print("\t");
    Serial.print(pitch3);
    Serial.print("\t");
    Serial.print(roll3);
    Serial.print("\t");
  
    Serial.print(claw1);
    Serial.print("\t");
  
    Serial.println(myData.id);

  }

  // id 2 --> left arm
  else if (myData.id == 2) {
    yaw4 = myData.yaw_6050_1;
    pitch4 = myData.pitch_6050_1;
    roll4 = myData.roll_6050_1;

    yaw5 = myData.yaw_6050_2;
    pitch5 = myData.pitch_6050_2;
    roll5 = myData.roll_6050_2;

    yaw6 = myData.yaw_9250-newYaw9;
    if (yaw6 < 0) {
      yaw6 += 360;
    } else if (yaw6 >= 360) {
      yaw6 -= 360;
    }
    pitch6 = myData.pitch_9250;
    roll6 = myData.roll_9250;

    claw2 = myData.claw;

    
    Serial.print(yaw4);
    Serial.print("\t");
    Serial.print(pitch4);
    Serial.print("\t");
    Serial.print(roll4);
    Serial.print("\t");
  
    Serial.print(yaw5);
    Serial.print("\t");
    Serial.print(pitch5);
    Serial.print("\t");
    Serial.print(roll5);
    Serial.print("\t");
  
    Serial.print(yaw6);
    Serial.print("\t");
    Serial.print(pitch6);
    Serial.print("\t");
    Serial.print(roll6);
    Serial.print("\t");
  
    Serial.print(claw2);
    Serial.print("\t");
  
    Serial.println(myData.id);  

  }
  
  // id 3 --> reference sensor
  if (myData.id == 3) {
    yaw7 = myData.yaw_6050_1;
    pitch7 = myData.pitch_6050_1;
    roll7 = myData.roll_6050_1;

    yaw8 = myData.yaw_6050_2;
    pitch8 = myData.pitch_6050_2;
    roll8 = myData.roll_6050_2;

    yaw9 = myData.yaw_9250;
    if(yaw9 < 0) {newYaw9 = yaw9 + 180 -25;}
    if(yaw9 >= 0) {newYaw9 = -(180 - yaw9) -25;}
    pitch9 = myData.pitch_9250;
    roll9 = myData.roll_9250;
//
//    Serial.print(yaw7);
//    Serial.print("\t");
//    Serial.print(pitch7);
//    Serial.print("\t");
//    Serial.print(roll7);
//    Serial.print("\t");
//  
//    Serial.print(yaw8);
//    Serial.print("\t");
//    Serial.print(pitch8);
//    Serial.print("\t");
//    Serial.print(roll8);
//    Serial.print("\t");
//  
//    Serial.print(newYaw9);
//    Serial.print("\t");
//    Serial.print(pitch9);
//    Serial.print("\t");
//    Serial.print(roll9);
//    Serial.print("\t");
//  
//    Serial.print(claw1);
//    Serial.print("\t");
//  
//    Serial.println(myData.id);

  }
  

  // Print all values together
  // Serial.println("Data received:");
  


  

  // Update the last receive time
  lastReceiveTime = millis();
  digitalWrite(led_indicator, HIGH); // Turn on LED when data is received
}


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);

  // Initialize the built-in LED
  pinMode(led_indicator, OUTPUT);
  digitalWrite(led_indicator, LOW); // Initially turn off the LED

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);


  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}


/**************************************************************************/
/*
    Arduino loop function
*/
/**************************************************************************/
void loop() {
  // Check if data has not been received for a certain amount of time
  if (millis() - lastReceiveTime > timeout) {
    // Blink the LED
    digitalWrite(led_indicator, !digitalRead(led_indicator));
    delay(200); // Blink interval
  }
}
