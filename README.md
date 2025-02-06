# GPS-Based Servo Control System

This project controls a servo motor based on GPS location data using an ESP32, TinyGPS++ library, and an MG996R servo motor. The servo is triggered when the device enters a predefined geofence area.

## Features
- Reads GPS coordinates from a Neo 8M GPS module.
- Defines geofence points with a specific radius.
- Calculates distance between current location and geofence points.
- Moves a servo motor to a predefined angle when inside the geofence.
- Returns the servo to its initial position when outside the geofence.

## Components Required
- **ESP32**
- **Neo 8M GPS module** (RX: 16, TX: 17)
- **MG996R Servo Motor** (Connected to pin 18)
- **Power source** (ESP32 and Servo motor)

## Circuit Connections
| Component | ESP32 Pin |
|-----------|----------|
| GPS RX    | 16       |
| GPS TX    | 17       |
| Servo PWM | 18       |



## Code Explanation
- Initializes GPS communication with a baud rate of `9600`.
- Reads real-time latitude and longitude values.
- Uses the **Haversine formula** to calculate distances.
- Checks if the current location is within `4 meters` of predefined geofence points.
- Moves the servo to `135°` if inside the geofence, otherwise remains at `90°`.

## Usage
1. Power on the ESP32 and GPS module.
2. Monitor GPS coordinates via the Serial Monitor (`115200 baud`).
3. Observe servo movement when entering geofence regions.

## Future Enhancements
- Implement dynamic geofencing via user input.
- Add OLED display for real-time location monitoring.
- Integrate Wi-Fi or Bluetooth for remote monitoring.

