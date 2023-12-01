
#include <Arduino.h>
#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#include <AsyncTCP.h>

#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#elif __has_include(<WiFiNINA.h>)
#include <WiFiNINA.h>
#elif __has_include(<WiFi101.h>)
#include <WiFi101.h>
#elif __has_include(<WiFiS3.h>)
#include <WiFiS3.h>
#endif

#include <ESP_Mail_Client.h>
#include <ESPAsyncWebServer.h>
#include "ThingSpeak.h"

// #define WIFI_SSID "Redmi Note 11T 5G"
// #define WIFI_PASSWORD "123jayesh456"

// #define WIFI_SSID "vivo V25"
// #define WIFI_PASSWORD "sid201260"

#define WIFI_SSID "Rohan's Galaxy F23 5G"
#define WIFI_PASSWORD "nhibtaunga"

AsyncWebServer server(80);

#define SMTP_HOST "smtp.gmail.com"


#define SMTP_PORT esp_mail_smtp_port_587 // port 465 is not available for Outlook.com

/* The log in credentials */
#define AUTHOR_EMAIL "firedetectioniotalert@gmail.com"
#define AUTHOR_PASSWORD "mnlbsqslgruqsizu"

/* Recipient email address */
#define RECIPIENT_EMAIL "sidag2020@gmail.com"

/* Declare the global used SMTPSession object for SMTP transport */
SMTPSession smtp;
const int motorPWM = 13;  
const int motorIN1 = 12;  

const int motorIN2 =14;  

int encoder_pin = 33;       // The pin the encoder is connected to
unsigned int rpm;           // RPM reading
volatile byte pulses;       // Number of pulses
// unsigned long timeold;
unsigned int pulsesperturn = 4;  // Pulses per revolution
// Define WiFi credentials
const char* ssid = "Redmi Note 11T 5G";   
const char* password = "123jayesh456";  



///////pid decl/////
double kp = 10;  // change values later
double ki = 0; // change values later
double kd = 0; // change values later
////////////////////////////

double setpoint = 255;       // VALUE SET BY THE USER
int prev_err = 0;            // previous error
int err;                     // error without proportionality constant
int eint = 0;                // integral error (summation of errors)
int eder;                    // derivative error
unsigned long prev_time = 0; // stores the time in the previous iteration of loop (in milliseconds)
int elapsed_time;            // stores time elapsed between current and previous reading observed (in seconds)
double u;                    // voltage to be applied after PID
int max_rpm;
///////pid end///////

// Initialize a WiFi client object
WiFiClient  client;

// ThingSpeak channel details
unsigned long myChannelNumber = 2235991;
const char * myWriteAPIKey = "L3GXA6X740NSJWI2";

// Timer variables
unsigned long lastTime = 0;
unsigned long timerDelay = 1000;
int curr_rpm = 255;
unsigned long timeold;
int i, sum;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
WiFiMulti multi;
#endif
/////////////  WEB PAGE PART //////////
const char* PARAM_INPUT_1 = "input1";
const char* PARAM_INPUT_2 = "input2";
const char* PARAM_INPUT_3 = "input3";
const char* PARAM_INPUT_4 = "input4";

// HTML web page to handle 3 input fields (input1, input2, input3)
// const char index_html[] PROGMEM = R"rawliteral(
// <!DOCTYPE HTML><html><head>
//   <title>ESP Input Form</title>
//   <meta name="viewport" content="width=device-width, initial-scale=1">
//   </head><body>
//   <form action="/get">
//     Enter RPM : <input type="text" name="input1">
//     <br>
//     Enter Kp : <input type="text" name="input2">
//     <br>
//     Enter Ki : <input type="text" name="input3">
//     <br>
//     Enter Kd : <input type="text" name="input4">
//     <br>
//     <input type="submit" value="Submit">
//   </form><br>
 
// </body></html>)rawliteral";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <!-- Include Chart.js library -->
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 20px;
    }

    form {
      max-width: 400px;
      margin: 0 auto;
      padding: 20px;
      border: 1px solid #ccc;
      border-radius: 5px;
      box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
    }

    label {
      display: block;
      margin-bottom: 8px;
    }

    input {
      width: 100%;
      padding: 8px;
      margin-bottom: 12px;
      box-sizing: border-box;
    }

    input[type="submit"] {
      background-color: #4caf50;
      color: white;
      cursor: pointer;
    }

    input[type="submit"]:hover {
      background-color: #45a049;
    }

    canvas {
      max-width: 100%;
      height: auto;
      margin-top: 20px;
    }
  </style>
</head>
<body>
  <form action="/get">
    <label for="rpm">Enter RPM :</label>
    <input type="text" name="input1" id="rpm" placeholder="Enter RPM">

    <label for="kp">Enter Kp :</label>
    <input type="text" name="input2" id="kp" placeholder="Enter Kp">

    <label for="ki">Enter Ki :</label>
    <input type="text" name="input3" id="ki" placeholder="Enter Ki">

    <label for="kd">Enter Kd :</label>
    <input type="text" name="input4" id="kd" placeholder="Enter Kd">

    <input type="submit" value="Submit">
  </form>

  <!-- Chart.js graph -->
  <canvas id="myChart"></canvas>

  <script>
    // Sample data for the chart
    var ctx = document.getElementById('myChart').getContext('2d');
    var myChart = new Chart(ctx, {
      type: 'bar',
      data: {
        labels: ['Label 1', 'Label 2', 'Label 3', 'Label 4'],
        datasets: [{
          label: 'Sample Data',
          data: [12, 19, 3, 5],
          backgroundColor: [
            'rgba(255, 99, 132, 0.2)',
            'rgba(54, 162, 235, 0.2)',
            'rgba(255, 206, 86, 0.2)',
            'rgba(75, 192, 192, 0.2)',
          ],
          borderColor: [
            'rgba(255, 99, 132, 1)',
            'rgba(54, 162, 235, 1)',
            'rgba(255, 206, 86, 1)',
            'rgba(75, 192, 192, 1)',
          ],
          borderWidth: 1,
        }],
      },
    });
  </script>
</body>
</html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

//////////////////////////////////////////////

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status);

void counter()
{
  // Update count
  pulses++;
  Serial.println("counter");
}



void sendEmail(){

  Session_Config config;

  /* Set the session config */
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;

  config.login.user_domain = F("127.0.0.1");

  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.gmt_offset = 3;
  config.time.day_light_offset = 0;

  SMTP_Message message;

  // /* Set the message headers */
  message.sender.name = F("ESP Mail");
  message.sender.email = AUTHOR_EMAIL;


  message.subject = F("Low RPM");
  message.addRecipient(F("Someone"), RECIPIENT_EMAIL);

  String textMsg = "Alert : Your Motor speed is less than < 100 RPM";
  message.text.content = textMsg;

  message.text.charSet = F("us-ascii");
  message.addHeader(F("Message-ID: <abcde.fghij@gmail.com>"));

  message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;

  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_low;

  if (!smtp.connect(&config))
  {
    MailClient.printf("Connection error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return;
  }

  if (!smtp.isLoggedIn())
  {
    Serial.println("Not yet logged in.");
  }
  else
  {
    if (smtp.isAuthenticated())
    {
      Serial.println("Successfully logged in.");
    }
    else
    {
      Serial.println("Connected with no Auth.");
    }
  }
  if (!MailClient.sendMail(&smtp, &message))
  {
    MailClient.printf("Error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
  }

  // while(1){}; 
}

void setup()
{

  Serial.begin(115200);

  smtp.debug(1);

  /* Set the callback function to get the sending results */
  smtp.callback(smtpCallback);
  MailClient.networkReconnect(true);

  Session_Config config;

  /* Set the session config */
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;

  config.login.user_domain = F("127.0.0.1");

  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.gmt_offset = 3;
  config.time.day_light_offset = 0;

  if( WiFi.status() != WL_CONNECTED )
  {
      Serial.print("Connecting to Wi-Fi");
      while (WiFi.status() != WL_CONNECTED)
      {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.print(".");
        delay(5000);
    #if defined(ARDUINO_RASPBERRY_PI_PICO_W)
        if (millis() - ms > 10000)
          break;
    #endif
      }
      Serial.println();
      Serial.print("Connected with IP: ");
      Serial.println(WiFi.localIP());
      Serial.println();
  }

  if (!smtp.connect(&config))
  {
    MailClient.printf("Connection error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return;
  }

  if (!smtp.isLoggedIn())
  {
    Serial.println("Not yet logged in.");
  }
  else
  {
    if (smtp.isAuthenticated())
    {
      Serial.println("Successfully logged in.");
    }
    else
    {
      Serial.println("Connected with no Auth.");
    }
  }
 
  /* Start sending Email and close the session */

  // to clear sending result log
  // smtp.sendingResult.clear();
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
  curr_rpm = 190;
  max_rpm = 190; 
  // setpoint = 0;
  // Set WiFi mode to station (client)
  WiFi.mode(WIFI_STA);

  // Initialize ThingSpeak communication using the WiFi client
  ThingSpeak.begin(client);
  i = 0;
  sum = 0;

   // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String input_rpm,strkp,strki,strkd;
    String input1,input2,input3,input4;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      input_rpm = request->getParam(PARAM_INPUT_1)->value();
      input1 = PARAM_INPUT_1;
    }
    // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
    if (request->hasParam(PARAM_INPUT_2)) {
      strkp = request->getParam(PARAM_INPUT_2)->value();
      input2 = PARAM_INPUT_2;
    }
    // GET input3 value on <ESP_IP>/get?input3=<inputMessage>
    if (request->hasParam(PARAM_INPUT_3)) {
      strki = request->getParam(PARAM_INPUT_3)->value();
      input3 = PARAM_INPUT_3;
    }
    if (request->hasParam(PARAM_INPUT_4)) {
      strkd = request->getParam(PARAM_INPUT_4)->value();
      input4 = PARAM_INPUT_4;
    }
   
    // else {
    //   inputMessage = "No message sent";
    //   inputParam = "none";
    // }
    Serial.print("Message received from server:\n");
    Serial.print("RPM:");
    Serial.print(input_rpm);
    Serial.print("\t");
    Serial.print("kp:");
    Serial.print(strkp);
    Serial.print("\t");
    Serial.print("ki:");
    Serial.print(strki);
    Serial.print("\t");
    Serial.print("kd:");
    Serial.print(strkd);
    Serial.print("\n");

    if( 0 <= input_rpm.toInt() && input_rpm.toInt() <= 540 )
    {
       curr_rpm = input_rpm.toInt();
       setpoint = curr_rpm;
      //  setpoint = curr_rpm*255/max_rpm;
    }
    if( 0 <= strkp.toFloat() && strki.toFloat() <= 100 )
    {
       kp = strkp.toFloat();
    }
    if( 0 <= strki.toFloat() && strki.toFloat() <= 100 )
    {
       ki = strki.toFloat();
    }
    if( 0 <= strkd.toFloat() && strkd.toFloat() <= 100 )
    {
       kd = strkd.toFloat();
    }
    // else
    // {
    //    Serial.println("Error:value out of bound");
    // }
    request->send(200, "text/html", "HTTP GET request sent to your ESP on input field <br><a href=\"/\">Return to Home Page</a>");
  });
  server.onNotFound(notFound);
  server.begin();
}

void loop()
{
   
#if defined(ARDUINO_ARCH_SAMD)
  while (!Serial)
    ;
#endif

  // Serial.println();

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  multi.addAP(WIFI_SSID, WIFI_PASSWORD);
  multi.run();
// #else
//   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif

  

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  unsigned long ms = millis();
#endif

if( WiFi.status() != WL_CONNECTED )
{
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      Serial.print(".");
      delay(5000);
  #if defined(ARDUINO_RASPBERRY_PI_PICO_W)
      if (millis() - ms > 10000)
        break;
  #endif
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();
}

  /*  Set the network reconnection option */

  // The WiFi credentials are required for Pico W
  // due to it does not have reconnect feature.
  MailClient.clearAP();
#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  MailClient.addAP(WIFI_SSID, WIFI_PASSWORD);
#endif

////////// OUR CODE STARTS HERE///////

  digitalWrite(motorIN1, HIGH);
  if( i == 15 )
  {
    int avg = sum/8;
    ThingSpeak.setField(3, static_cast<long>(avg));
    if( WiFi.status() == WL_CONNECTED )
    {
        Serial.println();
        Serial.print("Connected with IP: ");
        Serial.println(WiFi.localIP());
        Serial.println();
    }
    // Write data to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if(x == 200){
      Serial.print("Channel update successful woth average rpm : ");
      Serial.println(avg);
    }
    else{
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
    if (avg < 0) {
      sendEmail();
    }
    i = 0;
    sum = 0;
  }
  if(setpoint > 255)
  {
    Serial.print("new value set to:");
    Serial.println(setpoint);
    analogWrite(motorPWM, 255);
  }
  else{
    Serial.print("new value set to:");
    Serial.println(setpoint);
    analogWrite(motorPWM, setpoint);
  }
  // Check if the timer interval has passed
  if ((millis() - lastTime) > timerDelay) {

    // detachInterrupt(digitalPinToInterrupt(encoder_pin));
    // Calculate RPM
    rpm = (pulses * 60 * 1000 )/( pulsesperturn * (millis() - lastTime) );
    rpm = rpm / 2;
    timeold = millis();
    pulses = 0;
    if(setpoint > 255){
      Serial.print("Our input:");
      Serial.print(max_rpm);
      Serial.print("\t");
      Serial.print("Measured output:");
      Serial.print(rpm);
      Serial.print("\n");
    }
    else{
      Serial.print("Our input:");
      Serial.print(curr_rpm);
      Serial.print("\t");
      Serial.print("Measured output:");
      Serial.print(rpm);
      Serial.print("\n");
    }

    sum += rpm;      /// summing up the Measured values

    ////// pid start //////

    // err = curr_rpm - rpm;
    // elapsed_time = (millis() - prev_time)/1000;
    // eint += err * elapsed_time;
    // eder = (err - prev_err) / elapsed_time;
    // u = kp * err + ki * eint + kd * eder; // final calculated voltage

    // Serial.print("Proportional term: ");
    // Serial.println(kp*err);
    // Serial.print("Integral term: ");
    // Serial.println(ki*eint);
    // Serial.print("Derivative term: ");
    // Serial.println(kd*eder);

    // prev_err = err;
    // prev_time = millis();
   
    // setpoint = abs(u); // seting input(setpoint) to new calculated value

    // Serial.print("Setpoint: ");
    // Serial.println(setpoint);

    // if( setpoint > 255 )
    // {
    //   setpoint = 255;
    // }

    ////// pid end /////////

    // attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, FALLING);
    lastTime = millis();
  }
  delay(1000);
  i++;
}

void smtpCallback(SMTP_Status status)
{
  /* Print the current status */
  Serial.println(status.info());

  /* Print the sending result */
  if (status.success())
  {
    // MailClient.printf used in the examples is for format printing via debug Serial port
    // that works for all supported Arduino platform SDKs e.g. SAMD, ESP32 and ESP8266.
    // In ESP8266 and ESP32, you can use Serial.printf directly.

    Serial.println("----------------");
    MailClient.printf("Message sent success: %d\n", status.completedCount());
    MailClient.printf("Message sent failed: %d\n", status.failedCount());
    Serial.println("----------------\n");

    for (size_t i = 0; i < smtp.sendingResult.size(); i++)
    {
      /* Get the result item */
      SMTP_Result result = smtp.sendingResult.getItem(i);

      // In case, ESP32, ESP8266 and SAMD device, the timestamp get from result.timestamp should be valid if
      // your device time was synched with NTP server.
      // Other devices may show invalid timestamp as the device time was not set i.e. it will show Jan 1, 1970.
      // You can call smtp.setSystemTime(xxx) to set device time manually. Where xxx is timestamp (seconds since Jan 1, 1970)
      MailClient.printf("Message No: %d\n", i + 1);

      MailClient.printf("Status: %s\n", result.completed ? "success" : "failed");
      MailClient.printf("Date/Time: %s\n", MailClient.Time.getDateTimeString(result.timestamp, "%B %d, %Y %H:%M:%S").c_str());
      MailClient.printf("Recipient: %s\n", result.recipients.c_str());
      MailClient.printf("Subject: %s\n", result.subject.c_str());
    }
    Serial.println("----------------\n");

    // You need to clear sending result as the memory usage will grow up.
    smtp.sendingResult.clear();
  }
}
