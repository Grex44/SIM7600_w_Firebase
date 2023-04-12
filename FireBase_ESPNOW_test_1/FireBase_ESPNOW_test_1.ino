/*
  Rui Santos
  Complete project details at our blog.
    - ESP32: https://RandomNerdTutorials.com/esp32-firebase-realtime-database/
    - ESP8266: https://RandomNerdTutorials.com/esp8266-nodemcu-firebase-realtime-database/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  Based in the RTDB Basic Example by Firebase-ESP-Client library by mobizt
  https://github.com/mobizt/Firebase-ESP-Client/blob/main/examples/RTDB/Basic/Basic.ino
*/
#include <esp_now.h>
#include <Arduino.h>
// #if defined(ESP32)
#include <WiFi.h>
// #elif defined(ESP8266)
//   #include <ESP8266WiFi.h>
// #endif
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

// Insert your network credentials
// Replace with your network credentials (STATION)
#define WIFI_SSID "Little Pooh"
#define WIFI_PASSWORD "poohbear"
// #define WIFI_SSID "Performance Rotors"
// #define WIFI_PASSWORD "rotors0156"

// Insert Firebase project API Key
#define API_KEY "AIzaSyB3Sx5aEsllNJGFiTz16YdLsjwAbxwZ_bs"

// Insert RTDB URLefine the RTDB URL */
#define DATABASE_URL "https://esp32relaygomama-default-rtdb.asia-southeast1.firebasedatabase.app/"

//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
int count = 0;
bool signupOK = false;

//ESPNOW portion//

// REPLACE WITH YOUR RECEIVER MAC Address
// uint8_t broadcastAddress[] = { 0xA0, 0x76, 0x4E, 0x1B, 0xF7, 0xE8 };  //lolin c3 pico
uint8_t broadcastAddress[] = { 0x60, 0x55, 0xF9, 0x23, 0x43, 0x60 };      //Lolin C3 Mini v1.0.0    can work
// uint8_t broadcastAddress[] = { 0xA0, 0x76, 0x4E, 0x1D, 0x76, 0x84 };      //Lolin C3 Mini

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool occupied;
} struct_message;

// // Structure example to receive data
// // Must match the sender structure
// typedef struct struct_message {
//   int id;
//   // float temp;
//   // float hum;
//   int counter;
//   // bool occupied;
//   unsigned int readingId;
// } struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// JSONVar board;  //maybe no need this


////////////////////////////////
//Relays
#define Relay1 21
#define Relay2 19
#define Relay3 18
#define Relay4 5

//////////
//Indicator LED (RED)
#define IndLED    25

//////////////
//Human detection sensor
#define human1    4  //1st human sensor pin

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void InitESPNow() {
  //If the initialization was successful
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  //If there was an error
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

/*
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  // Copies the sender mac address to a string
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  
  board["id"] = incomingReadings.id;
  // board["temperature"] = incomingReadings.temp;
  // board["humidity"] = incomingReadings.hum;
  board["counter"] = incomingReadings.counter;
  board["readingId"] = String(incomingReadings.readingId);
  String jsonString = JSON.stringify(board);
  events.send(jsonString.c_str(), "new_readings", millis());
  
  Serial.printf("Board ID %u: %u bytes\n", incomingReadings.id, len);
  // Serial.printf("t value: %4.2f \n", incomingReadings.temp);
  // Serial.printf("h value: %4.2f \n", incomingReadings.hum);
  Serial.printf("counter %u: %u bytes\n", incomingReadings.counter, len);
  Serial.printf("readingID value: %d \n", incomingReadings.readingId);
  Serial.println();
}
*/


void setup() {
  Serial.begin(115200);

  pinMode(IndLED, OUTPUT);
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay4, OUTPUT);
  pinMode(human1, INPUT);

  LightOFF();

  myData.occupied = false;


  // Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_AP_STA);
  // WiFi.disconnect();

  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial);  // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial);  // Uncomment to verify channel change after

  delay(100);

  // Set device as a Wi-Fi Station
  Serial.println("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to Wi-Fi.");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Init ESP-NOW
  // if (esp_now_init() != ESP_OK) {
  //   Serial.println("Error initializing ESP-NOW");
  //   return;
  // }

  // Serial.println("Initializing ESP-NOW...");
  // // Init ESP-NOW
  // InitESPNow();

  // // Once ESPNow is successfully Init, we will register for Send CB to
  // // get the status of Trasnmitted packet
  // esp_now_register_send_cb(OnDataSent);

  // // Register peer
  // memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  // peerInfo.channel = 0;
  // peerInfo.encrypt = false;

  // // Add peer
  // if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  //   Serial.println("Failed to add peer");
  //   return;
  // }

  // //Blink twice to indicate ESPNOW is initallized.
  // Blink(2);

  Serial.println("Initializing Firebase...");
  //Config Firebase

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("ok");
    signupOK = true;
  } else {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback;  //see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);


  //Blink twice to indicate ESPNOW is initallized.
  Blink(2);


  Serial.println("Initializing ESP-NOW...");
  // Init ESP-NOW
  InitESPNow();

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
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

  Blink(3);
}

void loop() {
  bool humanSense1 = digitalRead(human1);  //maybe attached this as interrupt pin

  // Set values to send
  strcpy(myData.a, "THIS IS A CHAR");
  myData.b = random(1, 100);
  myData.c = random(0.0, 100.0);
  // myData.d = true;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
    delay(100);  // wait 100ms
  } else {
    Serial.println("Error sending the data");
    delay(100);
  }

  delay(10);

  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)) {  //change the time= 15000(15s) to shorter time. eg. 5s
    sendDataPrevMillis = millis();
    // Write an Int number on the database path test/int
    if (Firebase.RTDB.setInt(&fbdo, "test/int", count)) {
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
    } else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    count++;

    // //human sense and counter for it;

    if (humanSense1) {
      // myData.counter += 1;
      myData.occupied = true;
      LightON();
    } else {
      myData.occupied = false;
      LightOFF();
    }

    // Write an Float number on the database path test/float
    if (Firebase.RTDB.setFloat(&fbdo, "test/float", 0.01 + random(0, 100))) {
      Serial.println("PASSED");
      Serial.println("PATH: " + fbdo.dataPath());
      Serial.println("TYPE: " + fbdo.dataType());
      Blink(1);
    } else {
      Serial.println("FAILED");
      Serial.println("REASON: " + fbdo.errorReason());
    }
  }
}

void LightON(){
  digitalWrite(Relay1, HIGH);
  delay(10);
}

void LightOFF(){
  digitalWrite(Relay1, LOW);
  delay(10);
}

void Blink(int a) {
  for (int i = 0; i < a; i++) {
    digitalWrite(IndLED, HIGH);
    delay(400);
    digitalWrite(IndLED, LOW);
    delay(250);
  }
}