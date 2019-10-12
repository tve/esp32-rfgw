// Registry to keep track of RF nodes and which GW is responsible for their packets

#include <map>
#include <ArduinoJson.h>
#include <functional>
using namespace std::placeholders;

#ifndef MAX_GW
#define MAX_GW 10
#endif

class NodeRegistry {
public:

    // doAck returns true if an ACK should be sent, false otherwise.
    // It essentially checks the margin info received from all the gw for the previous packet
    bool doAck(uint32_t nodeId) {
        NodeEntry &e = nodes[nodeId]; // note: [] creates entry if it doesn't exist
        return e.gwId <= 0; // <0 => always reply to new nodes, may cause a collision...
    }

    // addInfo registers the margin info received from a gw.
    // It keeps track of the strongest signal, but replaces it if it is more than a few seconds old
    // in an attempt to keep track of only the lasst packet data.
    void addInfo(uint32_t nodeId, const char *gw, int margin) {
        int gwId = lookupGW(gw);
        NodeEntry &e = nodes[nodeId];
        if (e.gwId < 0 || margin > e.margin || millis()-e.at > entryTimeout) {
            e.gwId = gwId;
            e.margin = margin;
            e.at = millis();
            if (debug) printf("Node %08x reachable via GW %d with %ddB\n", nodeId, gwId, margin);
        }
    }

    bool debug = true;

//private:
    const char *gateways[MAX_GW];       // index 0 = self
    int numGW;
    uint32_t entryTimeout;              // timeout for the top entry, in ms

    struct NodeEntry {
        int16_t gwId;
        int16_t margin;
        uint32_t at;
        NodeEntry() : gwId(-1), margin(-100), at(millis()) {};
    };
    std::map<uint32_t, NodeEntry> nodes;

    NodeRegistry(const char *selfGwName) {
        gateways[0] = selfGwName;
        numGW = 1;
        entryTimeout = 5000;
    }

    void setSelf(const char *selfGwName) {
        gateways[0] = selfGwName;
    }

    int lookupGW(const char *gwName) {
        for (int i=0; i<numGW; i++) {
            if (strcmp(gwName, gateways[i]) == 0) return i;
        }
        int i = numGW < MAX_GW ? numGW++ : 1;
        char *copy = (char *)malloc(strlen(gwName)+1);
        strcpy(copy, gwName);
        gateways[i] = copy;
        if (debug) printf("New GW #%d: %s\n", i, copy);
        return i;
    }
};

class NodeRegistryWorker {
public:
    NodeRegistry registry;

    NodeRegistryWorker(const char *self, const char *gwTopic = "rfgw/reports",
            const char *ackTopic = "rfgw/acks")
        : registry(NodeRegistry(self))
        , gwTopic(gwTopic)
        , ackTopic(ackTopic)
        , selfGw(self)
    {}

    const char *gwTopic;
    const char *ackTopic;
    const char *selfGw;

    // onMqttMessage handles gw/node reception announcements.
    void onMqttMessage(char* topic, char* payload, MqttProps properties,
        size_t len, size_t index, size_t total)
    {
        // Handle gw/node announcements
        if (len < 128 && len == total && strcmp(topic, gwTopic) == 0) {
            DynamicJsonDocument json(256); // TODO: use JSON_OBJECT_SIZE, etc...
            DeserializationError err = deserializeJson(json, payload, len);
            if (err) {
                printf("Failed to deserialize %s message: %s\n", topic, err.c_str());
                return;
            }
            const char *gwName = json["gw"];
            int margin = json["margin"];
            uint32_t node = json["node"];
            bool ack = json["ack"];
            if (!gwName || node == 0) {
                printf("Bad data in %s message gwName=%s margin=%d node=%x\n", topic, gwName, margin, node);
                return;
            }
            if (!ack) registry.addInfo(node, gwName, margin);
        }
    }

    void onMqttConnect(bool sessionPresent) {
        registry.setSelf(selfGw);
        mqttClient.subscribe(gwTopic, 1);
        printf("Subscribed to %s for NodeRegistry\n", gwTopic);
    }

    bool doAck(uint32_t nodeId, int margin) {
        if (nodeId == 0) return false; // never ACK another GW's packet
        bool ack = registry.doAck(nodeId);
        char payload[128];
        snprintf(payload, 128, "{\"gw\":\"%s\",\"node\":%u,\"margin\":%d}", selfGw, nodeId, margin);
        uint16_t packetId = mqttClient.publish(gwTopic, 1, false, payload);
        if (registry.debug) printf("Pub to %s -> %d: %s\n", gwTopic, packetId, payload);
        if (ack) {
            snprintf(payload, 128, "{\"gw\":\"%s\",\"node\":%d,\"ack\":true}", selfGw, nodeId);
            mqttClient.publish(gwTopic, 1, false, payload);
        }
        return ack;
    }

    void setup() {
        mqttClient.onConnect(std::bind(&NodeRegistryWorker::onMqttConnect, *this, _1));
        mqttClient.onMessage(std::bind(&NodeRegistryWorker::onMqttMessage, *this,
                    _1, _2, _3, _4, _5, _6));
    }
};
