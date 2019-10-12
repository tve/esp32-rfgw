// ESP32 FSK Radio to MQTT gateway
// Copyright (c) 2019 Thorsten von Eicken, all rights reserved

#include <Arduino.h>
#include <SPI.h>
#include <SX1276fsk.h>
#include "formats.h"

jlPacket *processJLv1Pkt(uint8_t *buf, int len) {
    //printf("processJLv1Pkt, len=%d\n", len);
    if (len < 5) return 0;
    bool trailer = (buf[2] >> 7) & 1;
    //if (trailer && len < 9) return 0;
    uint8_t payLen = len - 3; // peel off hdr,src,fmt bytes
    if (trailer) payLen -= 2;
    jlPacket *pkt = (jlPacket *)calloc((sizeof(jlPacket)+payLen+3)/4, 4);
    bool fromGW = (buf[0]&0x3f) != 0;
    pkt->vers = 0; // 0->v1
    pkt->fromGW  = fromGW;
    pkt->isAck = fromGW;
    pkt->ackReq  = (buf[1] >> 7) & 1;
    pkt->fmt = buf[2] & 0x7f;
    pkt->trailer = trailer;
    if (pkt->trailer) {
        pkt->remMargin = buf[len-2] & 0x3f;
        pkt->remFEI = (((int8_t)buf[len-1])<<1>>1) * 128; // sign-extend
    }
    pkt->dataLen = 0;
    if (!pkt->fromGW && len >= 7) { // GW only sends acks and they have no data
        int l = decodeVarint(buf+3, 5, (int32_t*)&pkt->node); // varint decoding of node ID
        pkt->dataLen = payLen-l;
        memcpy(&pkt->data, buf+3+l, payLen-l); // data is stuff after node ID
    }
    return pkt;
}

jlPacket *processJLv2Pkt(uint8_t *buf, int len) {
    if (len < 6) return 0;
    bool trailer = (buf[5] >> 7) & 1;
    if (trailer && len < 8) return 0;
    uint8_t dataLen = len - 6;
    if (trailer) dataLen -= 2;
    jlPacket *pkt = (jlPacket *)calloc((sizeof(jlPacket)+dataLen+3)/4, 4);
    pkt->vers = 1; // 1->v2
    pkt->special = (buf[0] >> 6) & 1;
    pkt->fromGW  = (buf[0] >> 5) & 1;
    pkt->ackReq  = (buf[0] >> 4) & 1;
    pkt->fmt = buf[5];
    pkt->trailer = trailer;
    if (pkt->trailer) {
        pkt->remMargin = buf[len-2] & 0x3f;
        pkt->remFEI = (((int8_t)buf[len-1])<<1>>1) * 128; // sign-extend
    }
    memcpy(&pkt->node, buf+1, 4);
    pkt->dataLen = dataLen;
    memcpy(&pkt->data, buf+6, dataLen);
    return pkt;
}

jlPacket *processRFPacket(uint8_t *buf, int length, struct timeval rxAt, int8_t rssi, uint8_t snr, int16_t fei) {
    if (length < 2) return NULL;
    // heuristic to disambiguate old JL vs new JL formats
    jlPacket *pkt;
    if (((buf[0]&0x3f) == 0 && (buf[1]&0x3f) == 61) ||
        ((buf[1]&0x3f) == 0 && (buf[0]&0x3f) == 61)) {
            pkt = processJLv1Pkt(buf, length);
    } else {
            pkt = processJLv2Pkt(buf, length);
    }
    if (pkt) {
        pkt->at = rxAt;
        pkt->rssi = rssi;
        pkt->snr = snr;
        pkt->fei = fei;
        pkt->mqAt = 0;
    }
    return pkt;
}

