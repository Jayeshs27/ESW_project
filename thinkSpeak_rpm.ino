
#include <WiFi.h>
#include "ThingSpeak.h"

const int motorPWM = 19;  
const int motorIN1 = 21;  
const int motorIN2 = 22;  
int encoder_pin = 33;       // The pin the encoder is connected to
unsigned int rpm;           // RPM reading
volatile byte pulses;       // Number of pulses
// unsigned long timeold;
unsigned int pulsesperturn = 6;  // Pulses per revolution
// Define WiFi credentials
const char* ssid = "vivo V25";   
const char* password = "sid201260";  

// Initialize a WiFi client object
WiFiClient  client;

// ThingSpeak channel details
unsigned long myChannelNumber = 2235991;
const char * myWriteAPIKey = "L3GXA6X740NSJWI2";

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 1000;
unsigned long timeold;
int i, sum;

void counter()
{
  // Update count
  pulses++;
}

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200); 
  pinMode(encoder_pin, INPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorIN1, OUTPUT);
  pinMode(motorIN2, OUTPUT);
  
  digitalWrite(motorIN1, LOW);
  digitalWrite(motorIN2, LOW);
  
  ledcSetup(0, 5000, 8);
  ledcAttachPin(motorPWM, 0);
  // Attach an interrupt on digital pin 2 (interrupt number 0)
  // Triggers on FALLING (change from HIGH to LOW)
  attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, FALLING);
  pulses = 0;
  rpm = 0;
  // timeold = 0;
  timeold = 0;
  // Set WiFi mode to station (client)
  WiFi.mode(WIFI_STA);

  // Initialize ThingSpeak communication using the WiFi client
  ThingSpeak.begin(client);
  i = 0;
  sum = 0;
}

void loop() {
    if(WiFi.status() != WL_CONNECTED){
      Serial.print("Attempting to connect");
      while(WiFi.status() != WL_CONNECTED){
        WiFi.begin(ssid, password); 
        delay(5000);     
      } 
      Serial.println("\nConnected.");
    }
  digitalWrite(motorIN1, HIGH);
 
  if( i == 15 )
  {
    int avg = sum/8;
    ThingSpeak.setField(3, static_cast<long>(avg));

    // Write data to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if(x == 200){
      Serial.print("Channel update successful woth average rpm : ");
      Serial.println(avg);
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
    i = 0;
    sum = 0;
  }
  for(int i = 0 ; i < 255 ; i++ )
  {
     analogWrite(motorPWM, i);
  }
  // analogWrite(motorPWM, 255);

  // Check if the timer interval has passed
  if ((millis() - lastTime) > timerDelay) {
    
    // Connect or reconnect to WiFi if not connected


    detachInterrupt(digitalPinToInterrupt(encoder_pin));

    // Calculate RPM
    // rpm = (60 * 1000 / pulsesperturn) / (millis() - timeold) * pulses;
    rpm = (pulses * 60 * 1000 )/( pulsesperturn * (millis() - timeold) );
    timeold = millis();
    pulses = 0;
    // rpm/=5;
    Serial.println(rpm);
    sum += rpm;

    attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, FALLING);
    lastTime = millis();
  }
  delay(1000);
  i++;
}