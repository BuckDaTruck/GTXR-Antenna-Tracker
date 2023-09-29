
const float startingAltitude = 1000.0;  // Define the starting altitude
const int restartDelay = 5000;  // Delay in milliseconds before restarting the simulation

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Simulated GPS data with a changing flight path
  float altitude = startingAltitude + 100.0 * sin(millis() / 5000.0);  // Altitude varies sinusoidally
  float latitude = 40.0 + 0.1 * sin(millis() / 5000.0);  // Latitude varies sinusoidally
  float longitude = -75.0 + 0.2 * cos(millis() / 5000.0);  // Longitude varies cosinusoidally
  float velocity = 100.0 + 20.0 * sin(millis() / 5000.0);  // Velocity varies sinusoidally

  String gpsData = "@ GPS_STAT:Time:123456.789,Alt:" + String(altitude, 3) +
                   ",Lat:+" + String(latitude, 4) + ",Long:" + String(longitude, 4) +
                   ",Vel:" + String(velocity, 2) + ",Hhead:567.89,Uvel:10.0\r\n";

  // Send the data over serial
  Serial.print(gpsData);

  // Check if the altitude has crossed the starting altitude
  if (altitude >= startingAltitude) {
    // Delay before restarting the simulation
    delay(restartDelay);

    // Reset the time (millis) to start the simulation from the beginning
    millis(0);
  }

  // Delay for a while (simulating the GPS data transmission rate)
  delay(1000); // Adjust the delay as needed
}
