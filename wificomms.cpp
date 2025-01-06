#include "wificomms.h"

int pitch, int roll, int yaw, int throttle;

void wificomms::connect()
{
   pitch = roll = yaw = throttle = 0;

  WiFi.mode(WIFI_STA);
  WiFi.begin("homewifi", "pants-run-glad");
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  Serial.println(WiFi.localIP());
}

void wificomms::listen()
{
  UDP.begin(9999);
  
  udp.onPacket([](AsyncUDPPacket packet) {

      byte* b = packet.data();
      pitch = (int)*b++;
      roll = (int)*b++;
      yaw = (int)*b++;
      throttle = (int)*b;
    });
}

void wificomms::read(int& pitch_, int& roll_, int& yaw_, int& throttle_)
{
  pitch_ = pitch;
  roll_ = roll;
  yaw_ = yaw;
  throttle_ = throttle;
}
