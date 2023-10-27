// 4 Time -----------------------------------------------------------

IPAddress timeServer(129,6,15,28); //sets adress of time server; currently still using a nist.gov server, planning to change to npt.pool in near future to protect from server shutdowns 
byte messageBuffer[48]; //holds messages to/from NTP server

time_t getTime() {

    sendRequest(timeServer);
    uint32_t beginWait = millis();

    while (millis() - beginWait < 1500) {}
      int size = ethernet_UDP.parsePacket();

      if (size >= 48) {
        ethernet_UDP.read(messageBuffer, 48);

        unsigned long timeSince1900;

        timeSince1900 =  (unsigned long)messageBuffer[40] << 24;
        timeSince1900 |= (unsigned long)messageBuffer[41] << 16;
        timeSince1900 |= (unsigned long)messageBuffer[42] << 8;
        timeSince1900 |= (unsigned long)messageBuffer[43];

        return timeSince1900 - 2208988800UL; //returns epoch UTC time
      }
}

void sendRequest(IPAddress &address)
{
  memset(messageBuffer, 0, 48); //clears the messageBuffer

  //Create NTP message
  messageBuffer[0] = 0b11100011;
  messageBuffer[1] = 0;
  messageBuffer[2] = 6;
  messageBuffer[3] = 0xEC;
  // next 8 bytes left empty
  messageBuffer[12] = 49;
  messageBuffer[13] = 0x4E;
  messageBuffer[14] = 49;
  messageBuffer[15] = 52;

  //send the message to the NTP server via port 123
  ethernet_UDP.beginPacket(address, 123);
  ethernet_UDP.beginPacket(messageBuffer, 48);
  ethernet_UDP.endPacket();
}


// timeCheck
void timeCheck() {
  t = now();
  if (currentMs - msBackup > 5 * 60000) {
    tBackup = now();
    msBackup = millis();
    if (debugMode) {
      Serial.print("Back up time: ");
      Serial.println(tBackup);
    }
  }
  if (tStart) {
    tElapsed = t - tStart;
    tUnix = tUnixStart + tElapsed;
  }
}
