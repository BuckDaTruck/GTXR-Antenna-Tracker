const float startingAltitude = 1000.0;  // Define the starting altitude
const int restartDelay = 5000;  // Delay in milliseconds before restarting the simulation
unsigned long lastRestartTime = 0; // Track the time of the last restart

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Simulated GPS data with a changing flight path
  float altitude = startingAltitude + 100.0 * sin(millis() / 5000.0);  // Altitude varies sinusoidally
  float latitude = 33.0 + 0.001 * sin(millis() / 5000.0);  // Latitude varies sinusoidally
  float longitude = 33.0 + 0.003 * cos(millis() / 3000.0);  // Longitude varies cosinusoidally
  int velocity = 0 + 1 * sin(millis() / 5000.0);  // Velocity varies sinusoidally

  // Format the GPS data as a string
  String gpsData = "@ GPS_STAT 202 0000 00 00 00:04:09.294 CRC_OK TRK FthrWt02427 Alt " +
                   String(int(altitude), 6) + " lt +" + String(latitude, 8) + " ln +" + String(longitude, 8) +
                   " Vel " + String(velocity, 4) + " +000 +000 +0000 Fix 0 # 0 0 0 0 000_00_00 000_00_00 000_00_00 000_00_00 000_00_00 CRC: E13A";

  // Send the data over the Serial interface
  Serial.println(gpsData);

  // Check if the altitude has crossed the starting altitude
  if (altitude >= startingAltitude) {
    // Check if it's time to restart the simulation
    if (millis() - lastRestartTime >= restartDelay) {
      // Restart the simulation
      lastRestartTime = millis();
    }
  }

  // Delay for a while to control the data transmission rate
  delay(1000);  // Adjust the delay as needed
}
