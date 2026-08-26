void setup() {
  Serial.begin(115200);
  
  Serial.printf("Flash : %u MB\n", ESP.getFlashChipSize()/1024/1024);
  Serial.printf("PSRAM : %u MB\n", ESP.getPsramSize()/1024/1024);
}

void loop() {}