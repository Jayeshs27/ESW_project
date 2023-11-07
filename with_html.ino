
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

#define WIFI_SSID "vivo V25"
#define WIFI_PASSWORD "sid201260"
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
const int motorPWM = 19;  
const int motorIN1 = 21;  
const int motorIN2 = 22;  
int encoder_pin = 33;       // The pin the encoder is connected to
unsigned int rpm;           // RPM reading
volatile byte pulses;       // Number of pulses
// unsigned long timeold;
unsigned int pulsesperturn = 6;  // Pulses per revolution
// Define WiFi credentials
const char* ssid = "Redmi Note 11T 5G";   
const char* password = "123jayesh456";  

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

// HTML web page to handle 3 input fields (input1, input2, input3)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <form action="/get">
    input1: <input type="text" name="input1">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    input2: <input type="text" name="input2">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    input3: <input type="text" name="input3">
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";

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
  curr_rpm = 255;
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
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1;
    }
    // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_2)->value();
      inputParam = PARAM_INPUT_2;
    }
    // GET input3 value on <ESP_IP>/get?input3=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_3)) {
      inputMessage = request->getParam(PARAM_INPUT_3)->value();
      inputParam = PARAM_INPUT_3;
    }
    else {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    Serial.print("Message received from server:");
    Serial.println(inputMessage);
    if( 0 <= inputMessage.toInt() && inputMessage.toInt() <= 540 )
    {
       curr_rpm = inputMessage.toInt();
       curr_rpm = curr_rpm*255/540;
    }
    else
    {
       Serial.println("Error:value out of bound");
    }
    request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                     + inputParam + ") with value: " + inputMessage +
                                     "<br><a href=\"/\">Return to Home Page</a>");
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

////////// OURS CODE STARTS HERE///////

 digitalWrite(motorIN1, HIGH);
 
  if( i == 15 )
  {
    int avg = sum/15;
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
  // for(int i = 0 ; i < 255 ; i++ )
  // {
  //    analogWrite(motorPWM, i);
  //    delay(2000);
  // }
  // if( i%2 == 0 )
  // Serial.print("curr_rp1m: ");
  // Serial.println(curr_rpm);
  analogWrite(motorPWM, curr_rpm);
  // Serial.print("curr_rpm: ");
  // Serial.println(curr_rpm);
  // else
  //   analogWrite(motorPWM, 100);

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
    Serial.print("Our input:");
    Serial.print(curr_rpm*540/255);
    Serial.print("\t");
    Serial.print("Mesured output:");
    Serial.print(rpm);
    Serial.print("\n");

    sum += rpm;

    attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, FALLING);
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
