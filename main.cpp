// ESP32 Secure Base Basic Example
// Copyright (c) 2019 Thorsten von Eicken, all rights reserved

// Example application to demo Wifi and MQTT configuration using a commandline processor.
// It also supports OTA flash updates.

#include <Arduino.h>
#include <SPI.h>
#include <SX1276fsk.h>
#include <WiFi.h>
#include <ESPSecureBase.h>

//===== I/O pins/devices

SPIClass spi;
SX1276fsk radio(spi, 25, 27); // ss and reset pins

uint8_t rfId = 63; // 61=tx-only node, 63=promisc node
uint8_t rfGroup = 6;
uint32_t rfFreq = 912500;
int8_t rfPow = 10;
DV(rfId); DV(rfGroup); DV(rfFreq); DV(rfPow);

#define BUTTON 0
#ifndef LED
#define LED    1
#endif

ESBConfig config;
CommandParser cmdP(&Serial);
ESBCLI cmd(config, cmdP);
ESBDebug dbg(cmdP);

// MQTT message handling

void onMqttMessage(char* topic, char* payload, MqttProps properties,
    size_t len, size_t index, size_t total)
{
    // Handle over-the-air update messages
    if (strlen(topic) == mqTopicLen+4 && len == total &&
            strncmp(topic, mqTopic, mqTopicLen) == 0 &&
            strcmp(topic+mqTopicLen, "/ota") == 0)
    {
        ESBOTA::begin(payload, len);
    }
}

//===== Setup

void setup() {
    Serial.begin(115200);
    printf("\n===== ESP32 RF Gateway =====\n");

    config.read(); // read config file from flash
    cmd.init(); // init CLI
    mqttSetup(config);
    WiFi.mode(WIFI_STA); // start getting wifi to connect
    WiFi.begin();

    // radio init
    printf("Initializing radio\n");
    pinMode(25, OUTPUT);
    pinMode(27, OUTPUT);
    spi.begin(32, 35, 33);
    radio.init(rfId, rfGroup, rfFreq);
    radio.txPower(rfPow);

    printf("===== Setup complete\n");
}

uint32_t lastInfo = -1000000;
bool wifiConn = false;

void loop() {
    // print wifi/mqtt info every now and then
    bool conn = WiFi.isConnected();
    if (conn != wifiConn || millis() - lastInfo > 20000) {
        //WiFi.printDiag(Serial);
        printf("* Wifi:%s MQTT:%s\n",
                conn ? WiFi.SSID().c_str() : "---",
                mqttClient.connected() ? config.mqtt_server : "---");
        lastInfo = millis();
        wifiConn = conn;
    }

    static uint8_t pkt[256];
    int len = radio.receive(pkt, sizeof(pkt));
    if (len > 0) {
        printf("RX %02d->%02d %ddBm %dHz %d:",
                pkt[1]&0x3f, pkt[0]&0x3f, -radio.rssi/2, radio.afc, len);
        for (int i=2; i<len; i++) printf(" %02x", pkt[i]);
        printf("\n");
    }

    mqttLoop();
    cmd.loop();

    delay(10);
}
