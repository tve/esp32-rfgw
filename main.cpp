// ESP32 FSK Radio to MQTT gateway
// Copyright (c) 2019 Thorsten von Eicken, all rights reserved

// It also supports OTA flash updates.

#include <Arduino.h>
#include <SPI.h>
#include <SX1276fsk.h>
#include <WiFi.h>
#include <ESPSecureBase.h>
#include <libb64/cencode.h>
#include <lwip/apps/sntp.h>
#include "formats.h"
#include "registry.h"

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

#define LED_RED         16 // lit while there is no Wifi/MQTT connection
#define LED_GREEN       17 // pulsed for each MQTT message received
#define LED_BLUE        18 // pulsed for each RF packet received
uint32_t mqttLed = 0;

#define BUTTON 0      // unused

ESBConfig config;
CommandParser cmdP(&Serial);
ESBCLI cmd(config, cmdP);
ESBDebug dbg(cmdP);

#define GW_TOPIC "rfgw/reports"
bool mqttConn = false; // whether mqtt is connected or not
DV(mqttConn);

NodeRegistryWorker nrw(mqTopic, GW_TOPIC);

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

    digitalWrite(LED_GREEN, 0);
    mqttLed = millis();
}

void onMqttConnect(bool sessionPresent) {
    printf("Connected to MQTT, session %spresent\n", sessionPresent?"":"not ");
    char topic[128];

    strncpy(topic, mqTopic, 32);
    strcat(topic, "/ota");
    mqttClient.subscribe(topic, 1);
    printf("Subscribed to %s for OTA\n", topic);
}

// pktBuffer holds packets until they're acknowleged or we give up
static std::map<uint16_t, jlPacket*> pktBuffer;

void sendPacket(jlPacket *pkt, bool rexmit) {
    char buf[512];
    // rxAt is rx time in milliseconds since epoch (javascript timestamp)
    //uint64_t rxAt = (uint64_t)(pkt->at.tv_sec)*1000 + (uint64_t)(pkt->at.tv_usec)/1000;
    auto tm = gmtime(&pkt->at.tv_sec);
    // json-encode all the metadata
    int len = snprintf(buf, sizeof(buf),
            "{\"at\":\"%d-%02d-%02dT%02d:%02d:%02d.%03ldZ\","
             "\"gw\":\"%s\",\"hwid\":\"%x\",\"rssi\":%d,\"snr\":%d,\"fei\":%d,"
             "\"type\":%d,\"remote_margin\":%d,\"remote_fei\":%d,"
             "\"payload\":\"",
             tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec, pkt->at.tv_usec/1000,
             mqTopic, pkt->node, pkt->rssi, pkt->snr, pkt->fei,
             pkt->fmt, pkt->remMargin, pkt->remFEI);
    if (len == sizeof(buf)) {
        printf("OOPS: packet JSON too large\n");
        return;
    }
    // add the payload as a base64 encoded string
    if (pkt->dataLen > 0) {
        int paylen = base64_encode_expected_len(pkt->dataLen);
        if (len+paylen+2 >= sizeof(buf)) {
            printf("OOPS: packet JSON too large: need %d\n", len+paylen+2);
            return;
        }
        len += base64_encode_chars((char*)pkt->data, pkt->dataLen, buf+len);
        buf[len++] = '"';
    }
    // add the varint-decoded payload as data array
    if (pkt->dataLen > 0) {
        int32_t data[20];
        int c = decodeVarints(pkt->data, pkt->dataLen, data, 20);
        if (c > 0) {
            len += snprintf(buf+len, sizeof(buf)-len-2, ",\"data\":[");
            for (int i=0; i<c; i++)
                len += snprintf(buf+len, sizeof(buf)-len-2, "%d%c", data[i], i==c-1?']':',');
        } else {
            printf("Cannot decode varints: %d\n", c);
        }
    }
    buf[len++] = '}';
    buf[len  ] = 0;
    // send off the packet
    char topic[41+6];
    strcpy(topic, mqTopic);
    strcat(topic, "/rx");
    uint16_t id = mqttClient.publish(topic, 1, false, buf, len, rexmit);
    printf("Packet from %x at %ld %ssent, id=%d len=%d\n", pkt->node, pkt->at.tv_sec, rexmit?"re":"", id, len);
    //printf("JSON: %s\n", buf);
    pkt->mqAt = millis();
    pktBuffer[id] = pkt;
}

// onMqttPublish is called when a publish with QoS=1 succeeds. If the callback is for a packet
// in the buffer then remove that packet: we're done. Else ignore the callback.
void onMqttPublish(uint16_t id) {
    auto iter = pktBuffer.find(id);
    if (iter != pktBuffer.end()) {
        free(iter->second);
        pktBuffer.erase(iter);
    }
}

void packetLoop() {
    // check whether we should rexmit any packet
    if (mqttConn) {
        uint32_t now = millis();
        for (auto iter=pktBuffer.begin(); iter!=pktBuffer.end(); ) {
            jlPacket *pkt = iter->second;
            if (!pkt) { printf("OOPS: NULL pkt in pkBuffer at %d\n", iter->first); continue; }
            if (now - pkt->mqAt > 1100) {
                printf("Rexmit %d\n", iter->first);
                pktBuffer.erase(iter++);
                sendPacket(pkt, true);
            } else iter++;
        }
    }
    // check whether we need to drop some packets
    if (pktBuffer.size() > 100) {
        int drop = pktBuffer.size() - 100;
        struct timeval now;
        gettimeofday(&now, 0);
        printf("pktBuffer: dropping %d packets\n", drop);
        for (auto iter=pktBuffer.begin(); drop>0 && iter!=pktBuffer.end(); ) {
            jlPacket *pkt = iter->second;
            if (!pkt) { printf("OOPS: NULL pkt in pkBuffer at %d\n", iter->first); continue; }
            if (now.tv_sec - pkt->at.tv_sec > 5000) {
                free(pkt);
                pktBuffer.erase(iter++);
                drop--;
            } else iter++;
        }
    }
}

//===== Setup

void setup() {
    Serial.begin(115200);
    printf("\n===== ESP32 RF Gateway =====\n");
    printf("Running ESP-IDF %s\n", ESP.getSdkVersion());
    printf("Board type: %s\n", ARDUINO_BOARD);

    config.read(); // read config file from flash
    cmd.init(); // init CLI
    mqttSetup(config);
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.onPublish(onMqttPublish);
    WiFi.mode(WIFI_STA); // start getting wifi to connect
    WiFi.begin();
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    ip_addr_t sntpip = IPADDR4_INIT_BYTES(192, 168, 1, 1); // will be updated when wifi connects
    sntp_setserver(0, &sntpip);
    //sntp_servermode_dhcp(1); // not supported in arduino-esp32
    //sntp_setservername(1, (char *)"pool.ntp.org"); // esp-idf compiled with max 1 NTP server...
    sntp_init();

    // radio init
    printf("Initializing radio\n");
    spi.begin(RF_CLK, RF_MISO, RF_MOSI);
    radio.init(rfId, rfGroup, rfFreq);
    radio.setIntrPins(RF_DIO0, RF_DIO4);
    radio.txPower(rfPow);
    radio.setMode(SX1276fsk::MODE_STANDBY);

    nrw.setup();
    printf("pktBuf size = %d\n", pktBuffer.size());

    pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED, 0);
    pinMode(LED_BLUE, OUTPUT); digitalWrite(LED_BLUE, 1);
    pinMode(LED_GREEN, OUTPUT); digitalWrite(LED_GREEN, 1);
    delay(200);

    printf("===== Setup complete\n");
}

uint32_t lastInfo = -1000000;
bool wifiConn = false;
uint32_t lastReport = -50*1000;

void loop() {
    // print wifi/mqtt info every now and then
    bool conn = WiFi.isConnected();
    bool mqConn = mqttClient.connected();
    if (conn != wifiConn || mqConn != mqttConn || millis() - lastInfo > 20000) {
        //WiFi.printDiag(Serial);
        printf("* Wifi:%s %s | MQTT:%s\n",
                conn ? WiFi.SSID().c_str() : "---",
                WiFi.localIP().toString().c_str(),
                mqConn ? config.mqtt_server : "---");
        if (conn != wifiConn && conn) {
            // when we connect we set the SNTP server #0 to the IP address of the gateway, which
            // tends to be the NTP server on the LAN in 99% of cases. It would be nice if
            // arduino-esp32 supported DHCP discovery of the NTP serevr, but it doesn't...
            ip_addr_t sntpip = IPADDR4_INIT((uint32_t)(WiFi.gatewayIP()));
            sntp_setserver(0, &sntpip);
        }
        lastInfo = millis();
        wifiConn = conn;
        mqttConn = mqConn;
    }
    digitalWrite(LED_RED, (wifiConn && mqConn) ? 1 : 0);

    static uint8_t pktbuf[256];
    int len = radio.receive(pktbuf, sizeof(pktbuf));
    if (len > 0) {
        jlPacket *pkt = processRFPacket(pktbuf, len, radio.rxAt, -radio.rssi/2, radio.snr, radio.afc);
        if (pkt) {
            // decide whether to ACK and doit
            bool doAck = nrw.doAck(pkt->node, pkt->snr) && radio.rxAt.tv_sec;
            if (doAck) {
                uint8_t ack[5];
                if (pkt->vers == 0) {
                    ack[0] = 0; // dest node id, use bcast instead
                    ack[1] = 0x80; // fmt=0, got info trailer
                    ack[2] = (uint8_t)pkt->snr;
                    ack[3] = (uint8_t)(pkt->fei/128);
                    bool sent = radio.send(0x80, ack, 4);
                    if (!sent) printf("OOPS: couldn't sent ACK\n");
                } else {
                    // TODO: send ACK for v2 format
                }
            }
            // print packet debug info
            printf("RX %08x [%c%c%c%c%c]{%d} %ddBm %dHz",
                    pkt->node, pkt->isAck?'A':'.', pkt->fromGW?'>':'<',
                    pkt->ackReq?'Q':'.', pkt->special?'S':'.', pkt->trailer?'T':'.',
                    pkt->fmt, -radio.rssi/2, radio.afc);
            if (pkt->trailer) printf(" {%ddBm %dHz}", pkt->remMargin, pkt->remFEI);
            printf(" %d:", pkt->dataLen);
            for (int i=2; i<pkt->dataLen; i++) printf(" %02x", pkt->data[i]);
            if (doAck) printf(" --ACKED\n"); else printf("\n");
            // forward packet via MQTT
            if (pkt->node != 0 && radio.rxAt.tv_sec) {
                if (mqConn) {
                    sendPacket(pkt, false);
                } else {
                    pktBuffer[pkt->at.tv_usec & 0xffff] = pkt; // TODO: handle collisions
                }
            }
        } else {
            printf("Cannot decode packet:");
            for (int i=0; i<len; i++) printf(" %02x", pktbuf[i]);
            putchar('\n');
        }
    }

    if (millis() - lastReport > 10*1000) {
        time_t now;
        struct tm now_info;
        char now_str[64];
        auto *ip = sntp_getserver(0);
        time(&now);
        gmtime_r(&now, &now_info);
        strftime(now_str, sizeof(now_str), "%c", &now_info);
        if (now_info.tm_year > (2016-1900)) {
            printf("Time: %s (from %08x)\n", now_str, ip->u_addr.ip4.addr);
        } else {
            printf("The time is not available: %s (server %08x)\n", now_str, ip->u_addr.ip4.addr);
        }
        if (!sntp_enabled()) printf("SNTP not enabled!\n");
        //if (ip->u_addr.ip4.addr == 0) sntp_setservername(0, "pool.ntp.org");

        uint8_t pkt[50];
        for (int i=0; i<50; i++) pkt[i] = i;
        printf("Sending... ");
        bool sent = radio.send(0x80, pkt, 50);
        if (sent) {
            //while (radio.transmitting()) delayMicroseconds(10);
            //radio.receive(pkt, sizeof(pkt));
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
        } else { puts("send failed"); }
    }

    if (mqttLed != 0 && millis() - mqttLed > 200) {
        digitalWrite(LED_GREEN, 1);
        mqttLed = 0;
    }

    mqttLoop();
    cmd.loop();
    packetLoop();

    //delay(10);
}
