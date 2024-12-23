#include "wificomms.h"

void wificomms::connect()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin("homewifi", "pants-run-glad");
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.println(WiFi.localIP());
}

void wificomms::listen()
{
  UDP.begin(9999);
}

void wificomms::read(int& roll, int& pitch, int& throttle)
{
  char packet[4];
  int packetSize = UDP.parsePacket();
  if (packetSize) {
    int len = UDP.read(packet, 4);
    if (len > 0) { packet[len] = '\0'; }
    roll = int(packet[0]);
    pitch = int(packet[1]);
    throttle = int(packet[2]) * 24;
  }
}