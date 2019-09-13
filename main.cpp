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

#define RF_SS      25
#define RF_RESET   27
#define RF_CLK     32
#define RF_MISO    35
#define RF_MOSI    33
#define RF_DIO0    26
#define RF_DIO4    39

SPIClass spi;
SX1276fsk radio(spi, RF_SS, RF_RESET); // ss and reset pins

uint8_t rfId        = 63; // 61=tx-only node, 63=promisc node
uint8_t rfGroup     = 6;
uint32_t rfFreq     = 912500;
int8_t rfPow        = 10;
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
    spi.begin(RF_CLK, RF_MISO, RF_MOSI);
    radio.init(rfId, rfGroup, rfFreq);
    radio.setIntrPins(RF_DIO0, RF_DIO4);
    radio.txPower(rfPow);
    radio.setMode(SX1276fsk::MODE_STANDBY);


    pinMode(23, OUTPUT);
    digitalWrite(23, LOW);
    printf("===== Setup complete\n");
}

uint32_t lastInfo = -1000000;
bool wifiConn = false;
uint32_t lastReport = -50*1000;

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

    if (millis() - lastReport > 4*1000) {
        uint8_t pkt[50];
        for (int i=0; i<50; i++) pkt[i] = i;
        printf("Sending... ");
        digitalWrite(23, HIGH);
        bool sent = radio.send(0x80, pkt, 50);
        if (sent) {
            while (radio.transmitting()) delayMicroseconds(10);
            radio.receive(pkt, sizeof(pkt));
            digitalWrite(23, LOW);
            uint32_t s = millis();
            while (millis()-s < 30) {
                int len = radio.receive(pkt, sizeof(pkt));
                if (len > 0) {
                    printf("Got %d byte ACK in %ldms!", len, millis()-s);
                    break;
                }
            }
            puts("");
            lastReport = millis();
            digitalWrite(23, HIGH);
            delayMicroseconds(1000);
            digitalWrite(23, LOW);
        } else { puts("send failed"); }
    }

    mqttLoop();
    cmd.loop();

    //delay(10);
}
