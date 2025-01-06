#ifndef wificomms_h
#define wificomms_h

#include "WiFi.h"
#include "WiFiUdp.h"

class wificomms 
{
  public:
    void connect();
    void listen();
    void read(int& pitch, int& roll, int& yaw, int& throttle);

  private:

  WiFiUDP UDP;
};

#endif // wificomms_h