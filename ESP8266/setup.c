#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char *ssid     = "OPN3";
const char *password = "88888888";
const int udpPort    = 4210;

/* Hall sensor — connect signal wire to GPIO4 (D2 on most ESP-12 breakout boards) */
#define HALL_PIN 4

WiFiUDP udp;

volatile unsigned long g_lastEdge_us  = 0;
volatile unsigned long g_interval_us  = 0;
volatile bool          g_newPulse     = false;

/* Reject intervals shorter than this — suppresses noise/bounce on the signal line */
#define HALL_MIN_INTERVAL_US 3000UL

/* The disc magnet produces 1 pulse per wheel revolution (1 pole pair) */
#define POLES_PER_REV 1UL

void IRAM_ATTR hallISR()
{
    unsigned long now = micros();
    unsigned long dt  = now - g_lastEdge_us;
    if (g_lastEdge_us > 0 && dt >= HALL_MIN_INTERVAL_US)
    {
        g_interval_us = dt;
        g_newPulse    = true;
    }
    g_lastEdge_us = now;
}

void setup()
{
    Serial.begin(153600);

    pinMode(HALL_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, RISING);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
        delay(100);
    udp.begin(udpPort);
}

void loop()
{
    /* Hall → K66: send RPM over UART every 100 ms */
    static unsigned long lastRpmSend_ms = 0;
    unsigned long now_ms = millis();

    if (now_ms - lastRpmSend_ms >= 100)
    {
        lastRpmSend_ms = now_ms;

        noInterrupts();
        bool        fresh       = g_newPulse;
        unsigned long interval  = g_interval_us;
        unsigned long lastEdge  = g_lastEdge_us;
        g_newPulse = false;
        interrupts();

        int rpm = 0;
        /* Timeout: if last edge was more than 500 ms ago, wheel is stopped */
        if (fresh || (micros() - lastEdge) < 500000UL)
        {
            if (interval > 0)
            {
                /* Divide by pole count to get true wheel RPM */
                rpm = (int)(60000000UL / (interval * POLES_PER_REV));
            }
        }

        Serial.print("H:");
        Serial.print(rpm);
        Serial.print("\r");
    }

    /* K66 → Laptop: forward UART lines over UDP broadcast */
    if (Serial.available())
    {
        String line = Serial.readStringUntil('\n');
        udp.beginPacket("255.255.255.255", udpPort);
        udp.println(line);
        udp.endPacket();
    }

    /* Laptop → K66: forward incoming UDP packets to UART */
    int packetSize = udp.parsePacket();
    if (packetSize)
    {
        char buf[128];
        int len  = udp.read(buf, sizeof(buf) - 1);
        buf[len] = '\0';
        Serial.print(buf);
    }
}
