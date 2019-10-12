// ESP32 FSK Radio to MQTT gateway
// Copyright (c) 2019 Thorsten von Eicken, all rights reserved

// Jeelabs V1 format:
//   byte : content
//      0 : "header", 6-bits dest node ID, top 2 bits group parity
//      1 : "source", 6-bits source node ID, bit7: ack requested, bit6: unused
//      2 : format, 7-bit packet type, bit7: info trailer
// 3..6/7 : node_id, varint encoded
// 7..len : payload (7..len-2 if trailer present)
//   len-2: optional, 6 bit SNR/margin
//   len-1: optional, signed FEI/128
// ACK must start <10ms after packet end
//
// JeeLabs V2 format:
//   byte : content
//      0 : header, see below
//    1-4 : node ID (32 bits)
//      5 : format, 7-bit packet type, bit7: info trailer
// 6..len : payload (6..len-2 if trailer present)
//   len-2: optional, 6 bit SNR, margin above noise floor
//   len-1: optional, signed FEI/128
// ACK must start <10ms after packet end
// Header:
//   b8-b7  : group parity
//   b6 ctrl: 0=data 1=special.
//   b5 dest: 0=to-GW 1=from-GW.
//   b4 ack : 0=no-ack 1=ack-req.
//   b3-b2  : unused
//   b1-b0  : 0x2 to disambiguate from JLv1
// The following ctrl/ack combinations are used:
//   c=0, a=0 : data, no ack requested.
//   c=0, a=1 : data, ack requested.
//   c=1, a=0 : ack.
//   c=1, a=1 : unused.

// jlPacket contains a partially decoded JeeLabs v1 or v2 packet
struct jlPacket {
    bool        isAck:1, fromGW:1, ackReq:1, special:1, trailer:1, vers:2;
    uint8_t     fmt;            // 0..127
    int16_t     remFEI;         // in Hz
    uint8_t     remMargin;      // in dB
    uint8_t     dataLen;        // length of data array
    int16_t     fei;            // in Hz
    int8_t      rssi;           // in dBm
    uint8_t     snr;            // in dB
    struct timeval at;          // arrival timestamp
    uint32_t    mqAt;           // timestamp of last mqtt transmission
    uint32_t    node;           // 32-bit node id
    uint8_t     data[0];        // actual length is dataLen
};

jlPacket *processJLv1Pkt(uint8_t *buf, int len);
jlPacket *processJLv2Pkt(uint8_t *buf, int len);
jlPacket *processRFPacket(uint8_t *buf, int length, struct timeval rxAt, int8_t rssi,
        uint8_t snr, int16_t fei);
