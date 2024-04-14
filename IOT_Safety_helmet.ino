#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <DHT.h>

#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11

#define MQ2_PIN A0   // Analog pin connected to the MQ-2 sensor
#define BUZZER_PIN 11 // Digital pin connected to the buzzer
#define LED_PIN 13    // Digital pin connected to the LED

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Initialize the LCD: RS, E, D4, D5, D6, D7

const int sampleWindow = 50;  // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

#define SENSOR_PIN A0

#define TRIGGER_PIN 7 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 6    // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  pinMode(SENSOR_PIN, INPUT); // Set the signal pin as input
  pinMode(BUZZER_PIN, OUTPUT); // Set the buzzer pin as output
  pinMode(LED_PIN, OUTPUT);    // Set the LED pin as output

  Serial.begin(9600);

  lcd.begin();  // Initialize the LCD with 16 columns and 2 rows
  lcd.backlight();

  lcd.print("Loudness: ");  // Initial display message

  dht.begin(); // Initialize DHT sensor
}

void loop() {
  unsigned long startMillis = millis();                   // Start of sample window
  float peakToPeak = 0;                                  // peak-to-peak level

  unsigned int signalMax = 0;                            //minimum value
  unsigned int signalMin = 1024;                         //maximum value

  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow) {
    sample = analogRead(SENSOR_PIN);                    //get reading from microphone
    if (sample < 1024) {                                // toss out spurious readings
      if (sample > signalMax) {
        signalMax = sample;                           // save just the max levels
      } else if (sample < signalMin) {
        signalMin = sample;                           // save just the min levels
      }
    }
  }

  peakToPeak = signalMax - signalMin;                    // max - min = peak-peak amplitude
  int db = map(peakToPeak, 20, 900, 49.5, 90);             //calibrate for deciBels

  lcd.setCursor(0, 1);  // Set cursor to second row
  lcd.print("     "); // Clear previous value
  lcd.setCursor(0, 1);  // Set cursor to second row
  lcd.print(db);  // Print loudness value
  lcd.print(" dB");  // Print dB unit

  delay(200);

  // Ultrasonic sensor reading
  delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int distance = sonar.ping_cm(); // Send ping, get distance in centimeters and print result (0 = outside set distance range)
  lcd.setCursor(0, 0); // Set cursor to first row
  lcd.print("Dist: "); // Display label
  lcd.print(distance); // Display distance value
  lcd.print(" cm"); // Display unit

 if (distance <= 20 && distance!=0) {
    digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
    // Blink LED rapidly
    digitalWrite(LED_PIN, HIGH);
    delay(1);
    // digitalWrite(LED_PIN, LOW);
    // delay(5);
  } else {
    digitalWrite(BUZZER_PIN, LOW); // Turn off the buzzer
    digitalWrite(LED_PIN, LOW);    // Turn off the LED
  }
  
  delay(1000); // Delay for 1 second

  // MQ135 sensor reading
  float mq135Value = analogRead(A0); // Read analog value from MQ135 sensor
  float voltage = mq135Value * (5.0 / 1023.0); // Convert analog reading to voltage (assuming Arduino is powered with 5V)

  // Replace these values with your calibrated values for different gases
  // You'll need to calibrate the sensor according to the gas you want to detect
  float ppm = map(voltage, 0.4, 2.0, 0, 500); // Example calibration for CO2

  lcd.clear(); // Clear the LCD screen
  lcd.setCursor(0, 0); // Set cursor to first row
  lcd.print("Gas Conc: "); // Display label
  lcd.print(ppm); // Display gas concentration value
  lcd.print(" ppm"); // Display unit

  delay(1000); // Delay for 1 second

  // DHT11 sensor reading
  float humidity = dht.readHumidity(); // Read humidity value
  float temperature = dht.readTemperature(); // Read temperature value in Celsius

  lcd.clear(); // Clear the LCD screen
  lcd.setCursor(0, 0); // Set cursor to first row
  lcd.print("Temp:"); // Display label
  lcd.print(temperature); // Display temperature value
  lcd.print(" C"); // Display unit
  lcd.setCursor(0, 1); // Set cursor to second row
  lcd.print("Humidity: "); // Display label
  lcd.print(humidity); // Display humidity value
  lcd.print("%"); // Display unit

  delay(1000); // Delay for 1 second

  // MQ2 sensor reading
  int mq2Value = analogRead(MQ2_PIN); // Read analog value from MQ2 sensor
  float gasPercentage = (mq2Value / 1024.0) * 100.0; // Calculate gas percentage
  int airQualityIndex = map(mq2Value, 0, 1024, 0, 500); // Calculate air quality index

  lcd.clear(); // Clear the LCD screen
  lcd.setCursor(0, 0); // Set cursor to first row
  lcd.print("Air Quality: "); // Display label
  lcd.print(airQualityIndex); // Display air quality index
  lcd.setCursor(0, 1); // Set cursor to second row

  // Display gas percentage and check for lethal gas
  if (gasPercentage > 70.0) {
    lcd.print("Alert: Lethal Gas Detected");
    digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
  } else {
    lcd.print("Gas:");
    lcd.print(gasPercentage);
    lcd.print("%");
    digitalWrite(BUZZER_PIN, LOW); // Turn off the buzzer
  }
  

  // Check if the distance is less than or equal to 20 cm
 

  delay(1000); // Delay for 1 second
}