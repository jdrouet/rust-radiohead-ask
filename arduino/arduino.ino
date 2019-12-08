#include "RH_ASK.h"
#include <SPI.h>

#define RF_MESSAGE_SIZE 80
#define RF_NODE_ID 1
#define RF_SPEED 2000
#define RF_RX_PIN 11
#define RF_TX_PIN 12
#define SERIAL_SPEED 4800
#define SLEEP_DELAY 2000

RH_ASK driver(RF_SPEED, RF_RX_PIN, RF_TX_PIN, 0);

void send_ping() {
  char msg[RF_MESSAGE_SIZE];

  sprintf(msg, "hello %2d", RF_NODE_ID);

  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  Serial.println(msg);
}

void setup()
{
  Serial.begin(SERIAL_SPEED);
  if (!driver.init()) {
    Serial.println("init rf failed");
  }
}

void loop()
{
  send_ping();
  delay(SLEEP_DELAY);
}
