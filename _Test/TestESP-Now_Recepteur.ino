#include <WiFi.h>
#include <esp_now.h>

typedef struct __attribute__((packed))
{
    uint32_t nodeId;

    int32_t latitude;
    int32_t longitude;

    int16_t altitude;
    int16_t heading;

    uint8_t team;
    uint8_t etat;

    uint32_t timestamp;

} PositionPacket;

PositionPacket packet;

void onDataRecv(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int len)
{
  memcpy(&packet,
         data,
         sizeof(packet));

  Serial.println("------");

Serial.printf("Node      : %lu\n", packet.nodeId);

Serial.printf(
    "Latitude  : %.7f\n",
    packet.latitude / 10000000.0
);

Serial.printf(
    "Longitude : %.7f\n",
    packet.longitude / 10000000.0
);

Serial.printf(
    "Altitude  : %d m\n",
    packet.altitude
);

Serial.printf(
    "Heading   : %d deg\n",
    packet.heading
);

Serial.printf(
    "Equipe    : %u\n",
    packet.team
);

Serial.printf(
    "Etat  : %u %%\n",
    packet.etat
);

Serial.printf(
    "Timestamp : %lu\n",
    packet.timestamp
);

  for(int i=0;i<6;i++)
  {
    Serial.printf("%02X",
                  info->src_addr[i]);

    if(i<5)
      Serial.print(":");
  }

  Serial.println();
}

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
}

void loop()
{
}