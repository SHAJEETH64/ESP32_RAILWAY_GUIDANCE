#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>

// GPS Module Pins
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;

// Servo Settings
Servo servo;
const int servoPin = 18;
const int initialServoAngle = 90;

// Define Geofence Points
const double point1Lat = 10.943520;
const double point1Lon = 76.953871;
const double point2Lat = 10.943500;
const double point2Lon = 76.953909;

const double geofenceRadius = 4.0; // 1 meter radius

TinyGPSPlus gps;
HardwareSerial GPS(1);

void setup() {
  Serial.begin(115200);
  GPS.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  servo.attach(servoPin);
  servo.write(initialServoAngle); // Initial position
  delay(1000);

  Serial.println("GPS Servo Control Initialized...");
}

// Function to calculate distance between two coordinates
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000; // Earth's radius in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

void loop() {
  while (GPS.available()) {
    gps.encode(GPS.read());
  }

  if (gps.location.isUpdated()) {
    double currentLat = gps.location.lat();
    double currentLon = gps.location.lng();

    double distanceToPoint1 = calculateDistance(currentLat, currentLon, point1Lat, point1Lon);
    double distanceToPoint2 = calculateDistance(currentLat, currentLon, point2Lat, point2Lon);

    Serial.print("Latitude: "); Serial.println(currentLat, 6);
    Serial.print("Longitude: "); Serial.println(currentLon, 6);

    if (distanceToPoint1 <= geofenceRadius || distanceToPoint2 <= geofenceRadius) {
      servo.write(135); // Move to 135 degrees
      Serial.println("Servo Angle: 135");
      delay(5000); // Hold position for 5 seconds

      servo.write(90); // Return to initial position
      Serial.println("Servo Angle: 90");
    } else {
      servo.write(90); // Maintain initial position if outside geofence
      Serial.println("Servo Angle: 90");
    }
  }
  delay(1000); // Update every second
}
