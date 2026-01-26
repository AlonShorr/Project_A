#ifndef WIFI_LOGGER_H
#define WIFI_LOGGER_H

#include <Arduino.h>
#include <WiFi.h>

// --- CONFIG ---
#define WIFI_SSID "micromous"
#define WIFI_PASS "password" // Must be at least 8 chars
#define LOG_PORT 23

class WifiLogger {
public:
    void init() {
        // 1. Set Mode to Access Point (AP)
        WiFi.mode(WIFI_AP);
        
        // 2. Start the Access Point
        // The IP will usually be 192.168.4.1 by default
        if (WiFi.softAP(WIFI_SSID, WIFI_PASS)) {
            Serial.println("\n--- Wi-Fi AP Started ---");
            Serial.print("SSID: "); Serial.println(WIFI_SSID);
            Serial.print("IP Address: "); Serial.println(WiFi.softAPIP());
        } else {
            Serial.println("Wi-Fi AP Failed to start!");
        }

        server.begin();
        server.setNoDelay(true);
    }

    void handle() {
        // Check for new clients
        if (server.hasClient()) {
            if (!client || !client.connected()) {
                if (client) client.stop();
                // Use accept() for newer cores
                client = server.accept(); 
                client.println("--- Robot Log Connected ---");
            } else {
                // Reject if already connected (simple 1-client server)
                server.accept().stop();
            }
        }
    }

    // Like printf, but sends to Wi-Fi client
    void log(const char* format, ...) {
        char buffer[128];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        // Print to Serial (USB) as usual
        Serial.print(buffer);

        // Send to Telnet Client if connected
        if (client && client.connected()) {
            client.print(buffer);
        }
    }

private:
    WiFiServer server{LOG_PORT};
    WiFiClient client;
};

#endif