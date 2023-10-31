
#include <WiFi.h>
#include "ThingSpeak.h"

const int motorPWM = 19;
const int motorIN1 = 21;
const int motorIN2 = 22;
int encoder_pin = 33; // The pin the encoder is connected to
unsigned int rpm;     // RPM reading
volatile byte pulses; // Number of pulses
// unsigned long timeold;
unsigned int pulsesperturn = 6; // Pulses per revolution
// Define WiFi credentials
const char *ssid = "vivo V25";
const char *password = "sid201260";

double kp = 5;  // change values later
double ki = 10; // change values later
double kd = 20; // change values later

unsigned int setpoint;       // VALUE SET BY THE USER
int prev_err = 0;            // previous error
int err;                     // error without proportionality constant
int eint = 0;                // integral error (summation of errors)
int eder;                    // derivative error
unsigned long prev_time = 0; // stores the time in the previous iteration of loop (in microseconds)
double u;                    // voltage to be applied after PID

// Initialize a WiFi client object
WiFiClient client;

// ThingSpeak channel details
unsigned long myChannelNumber = 2235991;
const char *myWriteAPIKey = "L3GXA6X740NSJWI2";

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

void setup()
{
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

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("Attempting to connect");
        while (WiFi.status() != WL_CONNECTED)
        {
            WiFi.begin(ssid, password);
            delay(5000);
        }
        Serial.println("\nConnected.");
    }
    digitalWrite(motorIN1, HIGH);

    if (i == 15)
    {
        int avg = sum / 8;
        ThingSpeak.setField(3, static_cast<long>(avg));

        // Write data to the ThingSpeak channel
        int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
        if (x == 200)
        {
            Serial.print("Channel update successful woth average rpm : ");
            Serial.println(avg);
        }
        else
        {
            Serial.println("Problem updating channel. HTTP error code " + String(x));
        }
        i = 0;
        sum = 0;
    }
    for (int i = 0; i < 255; i++)
    {
        analogWrite(motorPWM, i);
    }
    // analogWrite(motorPWM, 255);

    // Check if the timer interval has passed
    if ((millis() - lastTime) > timerDelay)
    {

        // Connect or reconnect to WiFi if not connected

        detachInterrupt(digitalPinToInterrupt(encoder_pin));

        // Calculate RPM
        // rpm = (60 * 1000 / pulsesperturn) / (millis() - timeold) * pulses;
        rpm = (pulses * 60 * 1000) / (pulsesperturn * (millis() - timeold));
        timeold = millis();
        pulses = 0;
        // rpm/=5;
        Serial.println(rpm);
        sum += rpm;

        err = setpoint - rpm;
        eint += err;
        eder = (err - prev_err) * 1.0e6 / (micros() - prev_time);

        u = kp * e + ki * eint + kd * eder; // final calculated voltage

        prev_err = err;
        prev_time = micros();

        attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, FALLING);
        lastTime = millis();
    }
    delay(1000);
    i++;
}