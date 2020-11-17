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
#include "analog.h"

//===== I/O pins/devices

#if defined BOARD_RFGW2

#define RF_SS      25
#define RF_RESET   27
#define RF_CLK     32
#define RF_MISO    19
#define RF_MOSI    33
#define RF_DIO0    26
#define RF_DIO4    22

#define LED_MQTT    4 // pulsed for each MQTT message received
#define LED_RF      2 // pulsed for each RF packet received
#define LED_WIFI    LED_MQTT
#define LED_ON      1
#define LED_OFF     0

#define VBATT      35

#elif defined BOARD_EZSBC

#define RF_SS      25
#define RF_RESET   27
#define RF_CLK     32
#define RF_MISO    35
#define RF_MOSI    33
#define RF_DIO0    26
#define RF_DIO4    39

#define LED_WIFI   16 // lit while there is no Wifi/MQTT connection
#define LED_RF     17 // pulsed for each MQTT message received
#define LED_MQTT   18 // pulsed for each RF packet received
#define LED_ON      0
#define LED_OFF     1

#elif defined BOARD_HELTEC

#define RF_SS      18
#define RF_RESET   14
#define RF_CLK      5
#define RF_MISO    19
#define RF_MOSI    27
#define RF_DIO0    26
#define RF_DIO1    35
#define RF_DIO2    34
#define RF_DIO4    -1

#define LED        25 // pulsed for each RF packet or MQTT message received
#define LED_RF      LED
#define LED_MQTT    LED
#define LED_WIFI    LED
#define LED_ON      0
#define LED_OFF     1

#else

#error "Board is not defined"
#endif

SPIClass spi;
SX1276fsk radio(spi, RF_SS, RF_RESET); // ss and reset pins

uint8_t rfId        = 63; // 61=tx-only node, 63=promisc node
uint8_t rfGroup     = 6;
uint32_t rfFreq     = 912500000;
int8_t rfPow        = 17;
DV(rfId); DV(rfGroup); DV(rfFreq); DV(rfPow);

uint32_t rfLed = 0;
uint32_t mqttLed = 0;
uint32_t vBatt = 1;

ESBConfig config;
CommandParser cmdP(&Serial);
ESBCLI cmd(config, cmdP);
ESBDebug dbg(cmdP);

#define GW_TOPIC "rfgw/reports"
bool mqttConn = false; // whether mqtt is connected or not
DV(mqttConn);

NodeRegistryWorker nrw(mqTopic, GW_TOPIC);

// MQTT message handling

uint32_t mqttTxNum = 0, mqttRxNum = 0;

void onMqttMessage(char* topic, char* payload, MqttProps properties,
    size_t len, size_t index, size_t total)
{
    mqttRxNum++;

    // Handle over-the-air update messages
    if (strlen(topic) == mqTopicLen+4 && len == total &&
            strncmp(topic, mqTopic, mqTopicLen) == 0 &&
            strcmp(topic+mqTopicLen, "/ota") == 0)
    {
        ESBOTA::begin(payload, len);
    }

    digitalWrite(LED_WIFI, LED_ON);
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

// sendPacket forwards a packet to the MQTT broker
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
    printf("MQTT TX from %x at %ld %ssent, id=%d len=%d\n",
            pkt->node, pkt->at.tv_sec, rexmit?"re":"", id, len);
    //printf("JSON: %s\n", buf);
    pkt->mqAt = millis();
    pktBuffer[id] = pkt;
}

// onMqttPublish is called when a publish with QoS=1 succeeds. If the callback is for a packet
// in the buffer then remove that packet: we're done. Else ignore the callback.
void onMqttPublish(uint16_t id) {
    mqttTxNum++;
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
        printf("pktBuffer: %d packets left\n", pktBuffer.size());
    }
}

uint32_t rfTxNum = 0, rfRxNum = 0;

void rfLoop(bool mqConn) {
    static uint8_t pktbuf[70];
    int len = radio.receive(pktbuf, sizeof(pktbuf));
    if (len <= 0) return;
    rfRxNum++;
    digitalWrite(LED_RF, LED_ON);
    rfLed = millis();

    jlPacket *pkt = processRFPacket(pktbuf, len, radio.rxAt, -radio.rssi/2, radio.margin, radio.afc);
    // undecodable packet
    if (!pkt) {
        printf("Cannot decode packet:");
        for (int i=0; i<len; i++) printf(" %02x", pktbuf[i]);
        putchar('\n');
        return;
    }
    // packet from another GW - ignore
    if (pkt->node == 0) {
        printf("RF RX %08x [%c%c%c%c%c]{%d} %ddBm %dHz [Ignoring packet from another GW]\n",
                pkt->node, pkt->isAck?'A':'.', pkt->fromGW?'>':'<',
                pkt->ackReq?'Q':'.', pkt->special?'S':'.', pkt->trailer?'T':'.',
                pkt->fmt, -radio.rssi/2, radio.afc);
        return;
    }

    // decide whether to ACK and doit
    bool shouldAck = nrw.shouldAck(pkt->node) && radio.rxAt.tv_sec;
    if (shouldAck) {
        uint8_t ack[5];
        if (pkt->vers == 0) {
            ack[0] = 0x80; // fmt=0, got info trailer
            ack[1] = (uint8_t)pkt->snr;
            ack[2] = (uint8_t)(pkt->fei/128);
            delayMicroseconds(1000);
            bool sent = radio.send(61, ack, 3);
            if (!sent) printf("OOPS: couldn't sent ACK\n");
            else rfTxNum++;
        } else {
            // TODO: send ACK for v2 format
            printf("OOPS: V2 ACKs not implemented!\n");
        }
    }

    // send GW info via MQTT
    nrw.sendInfo(pkt->node, pkt->snr, shouldAck);

    // print packet debug info
    printf("RF RX %08x [%c%c%c%c%c]{%d} %ddBm %dHz",
            pkt->node, pkt->isAck?'A':'.', pkt->fromGW?'>':'<',
            pkt->ackReq?'Q':'.', pkt->special?'S':'.', pkt->trailer?'T':'.',
            pkt->fmt, -radio.rssi/2, radio.afc);
    if (pkt->trailer) printf(" {%ddBm %dHz}", pkt->remMargin, pkt->remFEI);
    printf(" %d:", pkt->dataLen);
    for (int i=2; i<pkt->dataLen; i++) printf(" %02x", pkt->data[i]);
    if (shouldAck) printf(" --ACKED\n"); else printf("\n");

    // forward packet via MQTT
    if (radio.rxAt.tv_sec) {
        if (mqConn) {
            sendPacket(pkt, false);
        } else {
            pktBuffer[pkt->at.tv_usec & 0xffff] = pkt; // TODO: handle collisions
        }
    }
}

extern uint32_t mqPingMs;

void report() {
    printf("vBatt = %dmV\n", vBatt);

    char buf[256];
    int len = snprintf(buf, sizeof(buf),
            "{\"uptime\":%d,\"rssi\":%d,\"heap\":%d,\"mVbatt\":%d,\"version\":\"%s\"",
            uint32_t(esp_timer_get_time()/1000000), WiFi.RSSI(), ESP.getFreeHeap(), vBatt, __DATE__);
    len += snprintf(buf+len, sizeof(buf)-len, ",\"rfTx\":%d,\"rfRx\":%d,\"rfNoise\":%d",
            rfTxNum, rfRxNum, -(radio.bgRssi>>5));
    len += snprintf(buf+len, sizeof(buf)-len,
            ",\"mqttTx\":%d,\"mqttRx\":%d,\"ping\":%d,\"queue\":%d",
            mqttTxNum, mqttRxNum, mqPingMs, pktBuffer.size());
    buf[len++] = '}';
    buf[len] = 0;

    // send off the packet
    char topic[41+6];
    strcpy(topic, mqTopic);
    strcat(topic, "/stats");
    mqttClient.publish(topic, 1, false, buf, len, false);
    printf("MQTT TX stats len=%d\n", len);
    //printf("JSON: %s\n", buf);
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

    pinMode(LED_MQTT, OUTPUT); digitalWrite(LED_MQTT, LED_OFF);
    pinMode(LED_RF, OUTPUT); digitalWrite(LED_RF, LED_OFF);
    pinMode(LED_WIFI, OUTPUT); digitalWrite(LED_WIFI, LED_ON);

#ifdef VBATT
    analogSetup(VBATT);
#endif

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
        if (mqttLed == 0) digitalWrite(LED_WIFI, (wifiConn && mqConn) ? LED_OFF : LED_ON);
    }

#ifdef VBATT
    uint32_t vB = 2 * analogSample(VBATT);
    vB = vB * 4046/4096;
    vBatt = (vBatt*15+vB)/16;
#endif

    rfLoop(mqConn);
    if (mqConn && millis() - lastReport > 20*1000) {
        report();
        lastReport = millis();
    }

    if (mqttLed != 0 && millis() - mqttLed > 200) {
        digitalWrite(LED_WIFI, LED_OFF);
        mqttLed = 0;
    }

    if (rfLed != 0 && millis() - rfLed > 200) {
        digitalWrite(LED_RF, LED_OFF);
        rfLed = 0;
    }

    mqttLoop();
    cmd.loop();
    packetLoop();

    //delay(10);
}
