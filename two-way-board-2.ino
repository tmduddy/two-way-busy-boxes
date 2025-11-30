/*
  great inspiration taken from:
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h>

// Receivers MAC address
uint8_t broadcastAddress[] = { 0x3C, 0x8A, 0x1F, 0x09, 0x35, 0x8C };

// LED pins
const int otherBusy = 13;
const int otherFree = 27;

const int selfBusy = 25;
const int selfFree = 32;

const int busyButton = 19;
const int freeButton = 23;

// Sync timing
const unsigned long SYNC_INTERVAL = 5UL * 60UL * 1000UL; // 5 minutes in milliseconds
// Stores the last time the boxes synced
unsigned long lastSyncTimeMs = 0; 

// State vars
bool isFree = true;
bool isBusy = false;

// Debug messaging
String sentStatusMessage;

//Structure to send data
//Must match the receiver structure
typedef struct struct_message {
  bool isBusy;
} struct_message;

// struct_message to hold incoming messages
struct_message incomingMsg;

bool incomingBusy;

// ESP-NOW
esp_now_peer_info_t peerInfo;

// Callback when data is sent, primarily for debugging
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    sentStatusMessage = "Delivery Success :)";
  } else {
    sentStatusMessage = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingBusy = incomingMsg.isBusy;
}

void sendData() {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&isBusy, sizeof(isBusy));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

void syncBoxes() {
  // Get the current time
  unsigned long currentTimeMs = millis();

  // Check if the interval has passed
  if (currentTimeMs - lastSyncTimeMs >= SYNC_INTERVAL) {
    lastSyncTimeMs = currentTimeMs; // Update the last action time

    // re-send the current pin statuses
    sendData();
  }
}

void setup() {
  // serial baud for the ESP-WROOM32 dev kits
  Serial.begin(115200);

  // init button pins with builtin pullup resistors
  pinMode(busyButton, INPUT_PULLUP);
  pinMode(freeButton, INPUT_PULLUP);

  //init LED pins using external resistors
  pinMode(otherBusy, OUTPUT);
  pinMode(otherFree, OUTPUT);
  pinMode(selfBusy, OUTPUT);
  pinMode(selfFree, OUTPUT);

  // enable WiFi to enable ESP-NOW
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
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
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  // Sync the boxes with their last sent statuses every five minutes.
  // This helps smooth over communication failures, which are more common
  // than I had expected.
  syncBoxes();


  // send data while button is actively pressed.
  if (digitalRead(busyButton) == LOW) {
    isBusy = true;
    isFree = false;
    sendData();
  }
  if (digitalRead(freeButton) == LOW) {
    isBusy = false;
    isFree = true;
    sendData();
  }

  // manage pin writes
  if (isBusy) {
    digitalWrite(selfBusy, HIGH);
    digitalWrite(selfFree, LOW);
  } else {
    digitalWrite(selfBusy, LOW);
    digitalWrite(selfFree, HIGH);
  }

  if (incomingBusy) {
    digitalWrite(otherBusy, HIGH);
    digitalWrite(otherFree, LOW);
  } else {
    digitalWrite(otherBusy, LOW);
    digitalWrite(otherFree, HIGH);
  }
}
