#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include "time.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>

/* 
Change Log
.4 added storage for json packet 
.5 added camera exception
.6 upload pump log data
.7 fixed most of the pause_time issue
.8 fixed the rest of the pause time issue and fixed the email not saving issue 
.9 fixed the feeding schedule timezone issue 
.91 logging
.92 default usb A to on, upload logs every few minute
.93 fixed web page start changed input_pullup to input, fixed current sensor added input
*/

// debug feed issue
unsigned long previousMillisFeedStatus = 0;  // will store the last time updated
const long intervalFeedStatus = 60000;       // interval at which to do something (60000 milliseconds = 1 minute)


// JSON packet
const char *json_packet = "{\"io_config\":[{\"io_pin\":26,\"on_time\":30000,\"pause_time\":500},{\"io_pin\":27,\"on_time\":100,\"pause_time\":500},{\"io_pin\":27,\"on_time\":100,\"pause_time\":500},{\"io_pin\":27,\"on_time\":100,\"pause_time\":500},{\"io_pin\":27,\"on_time\":100,\"pause_time\":500},{\"io_pin\":14,\"on_time\":300000,\"pause_time\":300000},{\"io_pin\":12,\"on_time\":30000,\"pause_time\":500},{\"io_pin\":13,\"on_time\":100,\"pause_time\":500},{\"io_pin\":13,\"on_time\":100,\"pause_time\":500},{\"io_pin\":13,\"on_time\":100,\"pause_time\":500},{\"io_pin\":13,\"on_time\":100,\"pause_time\":500}],\"feeding_config\":[{\"start_time\":\"123000AM\",\"io_pin\":27,\"on_time\":0,\"pause_time\":0},{\"start_time\":\"112010PM\",\"io_pin\":13,\"on_time\":500,\"pause_time\":0}]}";

const unsigned long downloadInterval = 3600000;
unsigned long lastDownload = 0;
DynamicJsonDocument doc(2048);

// feeding control so only one pump at a time
bool feed_pump_active = false;
bool io_pinActive = false;
bool pauseActive = false;
bool pauseStart = false;

// initialize preferences(where we store email)
Preferences preferences;
const char *emailKey = "email";
const char *jsonKey = "json";  // New constant for JSON storage key
const char *defaultEmail = "example@example.com";
char storedEmail[50];
bool get_email = false;
bool setup_failed = false;

// new email
String email = "test@email.com";

// initialize wifi manager
WiFiManager wm;
int timeout = 120;  // seconds to run for

// chip id
String my_chip_id;
char chip_id[23];
char output[23];

// select which pin will trigger the configuration portal when set to LOW
#define TRIGGER_PIN 18

// gentle machines pinout
const int led = 19;
const int pump1 = 26;
const int pump2 = 27;
const int pump3 = 14;
const int pump4 = 12;
const int pump5 = 13;
const int sensor1 = 25;
const int button = 18;
const int camera = 4;
const int usbtx2 = 17;
const int usbrx2 = 16;
const int readV = 35;  // voltage sensor pin
const int readI = 32;  // amp sensor pin

// logging power data
struct LogData {
  int io_pin;
  float ma;
  float mv;
  int pin_state;     // New member to store the pin state
  String timestamp;  // String to store the timestamp in the format "YYYY-MM-DD HH:II:SS"
};

std::vector<LogData> logArray;

// time - NTP server
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 0;
//char localTimezone = 'GMT-0';
unsigned long previousTMillis = 0;
const long intervalT = 60000;
char *get_local_time(void);

// current and voltage
#define VOLTAGE_PIN 35
#define CURRENT_PIN 32
#define ADC_RESOLUTION 4095  // Resolution of the ADC. ESP32 has a 12-bit ADC.
#define VREF_MV 3300         // Voltage reference for the ADC in millivolts. ESP32 typically has a voltage reference of 3300 mV.

#define CURRENT_MIN 0     // minimum current that sensor can measure in mA
#define CURRENT_MAX 1000  // maximum current that sensor can measure in mA

#define SAMPLE_PERIOD 1000  // Sample period in milliseconds
#define NUM_SAMPLES 10      // Number of samples to average
#define ALPHA 0.1           // Smoothing factor for EMA filter

float VOLTAGE_FACTOR = 4.4;  // Set voltage factor to 4.4
float CURRENT_FACTOR = 1.0;  // Start with the current factor of 1.0
float filteredCurrent = 0.0;
float voltage = 0.0;

unsigned long lastVoltageCurrentDisplayTime = 0;    // Last time the voltage and current were displayed
unsigned long voltageCurrentDisplayPeriod = 60000;  // Display voltage and current every 60 seconds (60000 milliseconds)


void setup() {
  Serial.begin(115200);
  Serial.println("Camera and pump3 off.");
  // turn on camera
  pinMode(camera, OUTPUT);
  digitalWrite(camera, LOW);
  // turn off pumps
  pinMode(pump1, OUTPUT);
  digitalWrite(pump1, LOW);
  pinMode(pump2, OUTPUT);
  digitalWrite(pump2, LOW);
  pinMode(pump3, OUTPUT);
  digitalWrite(pump3, LOW);
  pinMode(pump4, OUTPUT);
  digitalWrite(pump4, LOW);
  pinMode(pump5, OUTPUT);
  digitalWrite(pump5, LOW);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // setup screen
  pinMode(TRIGGER_PIN, INPUT);

  // water level sensor
  pinMode(sensor1, INPUT);

  // current sensor
  pinMode(readI, INPUT);

  // chip id
  snprintf(chip_id, 23, "%llX", ESP.getEfuseMac());
  everyTwoCharacters(chip_id, output);
  my_chip_id = String(output);
  Serial.print("my id: ");
  Serial.println(my_chip_id);

  WiFi.mode(WIFI_STA);  // explicitly set mode, esp defaults to STA+AP
  wm.setConfigPortalTimeout(1);
  wm.autoConnect();

  // Initialize preferences
  preferences.begin("myApp", false);
  // Retrieve the stored string
  email = preferences.getString("email", "default@email.com");  // the second parameter is a default value
  Serial.print("stored email: ");
  Serial.println(email);

  // Check if json_packet exists in non-volatile memory
  if (preferences.isKey(jsonKey)) {
    // Retrieve json_packet from non-volatile memory
    size_t len = preferences.getBytesLength(jsonKey);
    char *storedJson = new char[len];
    preferences.getBytes(jsonKey, (void *)storedJson, len);
    Serial.print("Retrieved JSON from non-volatile memory: ");
    Serial.println(storedJson);
    deserializeJson(doc, storedJson);  // Update doc with stored json_packet
    delete[] storedJson;               // Don't forget to delete allocated memory
  } else {
    // Load default json_packet
    deserializeJson(doc, json_packet);
  }

  JsonArray io_config = doc["io_config"];
  JsonArray feeding_config = doc["feeding_config"];

  if (WiFi.status() == WL_CONNECTED) {
    // Set NTP time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  }

  // Check if json_packet exists in non-volatile memory
  if (preferences.isKey(jsonKey)) {
    // Retrieve json_packet from non-volatile memory
    String storedJsonPacket = preferences.getString(jsonKey);
    // Use the stored json_packet
    deserializeJson(doc, storedJsonPacket);
  } else {
    // If no json_packet in memory, use the default one
    deserializeJson(doc, json_packet);
  }

  // current and voltage

  // Configure ADC
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);
  analogSetPinAttenuation(CURRENT_PIN, ADC_11db);

  // Set ADC resolution
  analogReadResolution(12);
}

void loop() {
  JsonArray io_config = doc["io_config"];
  JsonArray feeding_config = doc["feeding_config"];

  unsigned long currentMillis = millis();
  static int io_index = 0;
  static unsigned long io_previousMillis = 0;
  static unsigned long pause_previousMillis = 0;

  JsonObject io = io_config[io_index];
  int io_pin = io["io_pin"];
  int on_time = io["on_time"];
  int pause_time = io["pause_time"];
  int this_pause_time = 0;

  for (JsonObject feed : feeding_config) {
    const char *start_time = feed["start_time"];
    String startTime = String(start_time);

    if (startTime == getTimeString()) {
      feed_pump_active = true;
      // pump off
      digitalWrite(io_pin, LOW);

      // led off
      digitalWrite(led, LOW);
      io_pinActive = false;
      pauseActive = false;
      pauseStart = false;
      break;
    }
  }

  // pump on / off
  if (!feed_pump_active && !pauseStart) {
    //if (!io_pinActive && currentMillis - io_previousMillis >= pause_time) {
    if (!io_pinActive) {
      // turn on this pump
      Serial.print("circulation pump on: ");
      Serial.println(io_pin);

      digitalWrite(io_pin, HIGH);
      io_previousMillis = currentMillis;
      io_pinActive = true;
    } else if (io_pinActive && currentMillis - io_previousMillis >= on_time) {
      // turn off this pump
      addToLogArray(io_pin, filteredCurrent, voltage);
      addToLogArray(sensor1, 0, 0);  // water level
      Serial.print("circulation pump off: ");
      //Serial.print(filteredCurrent);
      Serial.println(io_pin);
      digitalWrite(io_pin, LOW);
      io_previousMillis = currentMillis;
      io_pinActive = false;
      //io_index = (io_index + 1) % io_config.size();
      pauseStart = true;
    }
  }

  // pause start and pause stop
  if (pauseStart == true) {
    if (!pauseActive) {
      unsigned long currentMillis = millis();
      // turn on this pump
      Serial.print("pause start: ");
      Serial.print(io_index);
      Serial.print(" / ");
      Serial.print(io_pin);
      Serial.print(" / ");
      Serial.println(pause_time);

      digitalWrite(led, HIGH);
      pause_previousMillis = currentMillis;
      pauseActive = true;
      //this_pause_time = pause_time;
    } else if (pauseActive && currentMillis - pause_previousMillis >= pause_time) {
      // turn on this pump
      Serial.print("pause stop: ");
      Serial.println(io_pin);
      digitalWrite(led, LOW);
      //pause_previousMillis = currentMillis;
      //io_pinActive = false;
      io_index = (io_index + 1) % io_config.size();
      pauseStart = false;
      pauseActive = false;
    }
    //pauseStart = false;
  }

  static unsigned long feed_previousMillis = 0;
  static bool feed_pinActive = false;
  static int active_feed_io_pin = -1;

  unsigned long currentMillisFeedStatus = millis();  // Get the current elapsed time

  for (JsonObject feed : feeding_config) {
    const char *start_time = feed["start_time"];
    String startTime = String(start_time);
    int feed_io_pin = feed["io_pin"];
    int feed_on_time = feed["on_time"];
    int feed_pause_time = feed["pause_time"];

    String feedTime = getTimeString();

    // If one minute has passed
    if (currentMillisFeedStatus - previousMillisFeedStatus >= intervalFeedStatus) {
      previousMillisFeedStatus = currentMillisFeedStatus;  // Remember the time
                                                           //Serial.println("One minute has passed!"); // Print message to the serial monitor
      // Serial.print(" now time: ");
      // Serial.println(feedTime);
      // Serial.print("feed time: ");
      // Serial.println(startTime);
    }

    if (startTime == feedTime && !feed_pinActive) {
      // turn off other pumps
      for (JsonObject io : io_config) {
        int io_pin = io["io_pin"];
        if (io_pin != feed_io_pin) {
          if (feed_io_pin == 4) {
            digitalWrite(io_pin, HIGH);
          } else {
            digitalWrite(io_pin, LOW);
          }
        }
      }

      // turn on this pump
      Serial.print("feed pump on: ");
      Serial.println(feed_io_pin);


      digitalWrite(feed_io_pin, HIGH);
      delay(10);
      feed_previousMillis = currentMillis;
      feed_pinActive = true;
      active_feed_io_pin = feed_io_pin;
    } else if (feed_pinActive && currentMillis - feed_previousMillis >= feed_on_time && active_feed_io_pin == feed_io_pin) {
      // turn off this pump
      addToLogArray(feed_io_pin, filteredCurrent, voltage);
      addToLogArray(sensor1, 0, 0);  // water level
      Serial.print("feed pump off: ");
      Serial.println(feed_io_pin);
      digitalWrite(feed_io_pin, LOW);
      delay(10);
      feed_previousMillis = currentMillis;
      feed_pinActive = false;
      active_feed_io_pin = -1;
      // reenable normal circulation
      feed_pump_active = false;
    }
  }

  // is configuration portal requested?
  if (digitalRead(TRIGGER_PIN) == LOW || setup_failed == true) {
    Serial.println("web portal button pushed!");
    // turn off pumps
    for (JsonObject io : io_config) {
      int io_pin = io["io_pin"];
      digitalWrite(io_pin, LOW);
    }
    WiFiManager wm;
    //wm.resetSettings();
    // set configportal timeout
    wm.setConfigPortalTimeout(timeout);
    setup_failed = false;
    // setup email field in wifi form
    WiFiManagerParameter email_address("email_address", "Your Email", storedEmail, 50);
    if (get_email == false) {
      get_email = true;
      // initialize wifiManager
      //WiFiManager wm;
      wm.addParameter(&email_address);
    }

    if (!wm.startConfigPortal()) {
      Serial.println("failed to connect and hit timeout");
      //setup_failed = true;
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }

    // get email entered on form
    Serial.print("email: ");
    Serial.println(email_address.getValue());
    preferences.putString(emailKey, email_address.getValue());
    Serial.print("copy: ");
    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
    downloadJson();
  }

  // download new directions once per hour
  if (WiFi.status() == WL_CONNECTED) {
    unsigned long currentDMillis = millis();
    if (currentDMillis - lastDownload >= downloadInterval || lastDownload == 0) {
      lastDownload = currentDMillis;
      Serial.println("Updating pump schedule.");
      downloadJson();
      Serial.print("my ip: ");
      Serial.println(WiFi.localIP());
      // ReSet NTP time
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    }
  }

  // current and voltage
  int rawVoltage = analogRead(VOLTAGE_PIN);
  filteredCurrent = readCurrentAverage();

  // Convert the raw readings to voltage and current
  voltage = ((float)rawVoltage / ADC_RESOLUTION * VREF_MV) * VOLTAGE_FACTOR;  // Apply the multiplication factor

  // Apply the current factor to the current reading
  filteredCurrent *= CURRENT_FACTOR;

  // Print the readings to the serial monitor every minute
  if (millis() - lastVoltageCurrentDisplayTime >= voltageCurrentDisplayPeriod) {

    Serial.print("mV / mA / pT / oT: ");
    Serial.print(voltage);
    Serial.print(" / ");
    Serial.print(filteredCurrent);
    Serial.print(" / ");
    Serial.print(pause_time);
    Serial.print(" / ");
    Serial.println(on_time);


    lastVoltageCurrentDisplayTime = millis();
  }

  // print time once per minute
  unsigned long currentTMillis = millis();
  if (currentTMillis - previousTMillis >= intervalT) {
    previousTMillis = currentTMillis;





    // Print the current time
    printLocalTime();
    String jsonPacket = json_log();
    if (!jsonPacket.isEmpty()) {
      Serial.print(jsonPacket);
      postJsonLogToServer(jsonPacket);
      deleteLogArray();
    }
  }
}

void downloadJson() {
  HTTPClient http;
  http.begin("https://mygrowbot.dwalliance.com/mygrowbot/json.php");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String postData = "?&control_board_id=" + my_chip_id + "&email=" + email;
  Serial.print("url foo:");
  Serial.println(postData);
  int httpCode = http.POST(postData);

  String payload = http.getString();

  if (httpCode > 0) {
    Serial.print("Downloaded:");
    Serial.println(payload);

    // Check if the newly downloaded payload is different from the stored one
    if (payload != preferences.getString(jsonKey)) {
      // Store the new payload in non-volatile memory
      preferences.putString(jsonKey, payload);
      Serial.println("Updated json_packet in non-volatile memory.");
    }

    // Use the new payload
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      deserializeJson(doc, json_packet);
    }
  } else {
    Serial.print("HTTP error: ");
    Serial.println(httpCode);
    deserializeJson(doc, json_packet);
  }

  http.end();
}


// String getTimeString() {
//   const char *format = "%I%M%S%p";
//   time_t now;
//   struct tm timeinfo;

//   time(&now);
//   localtime_r(&now, &timeinfo);
//   char buffer[9];

//   strftime(buffer, sizeof(buffer), format, &timeinfo);
//   return String(buffer);
// }

String getTimeString() {
  struct tm timeinfo;
  String timeStr;
  if (getLocalTime(&timeinfo)) {
    char buffer[9];
    sprintf(buffer, "%02d%02d%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    timeStr = String(buffer);
  } else {
    timeStr = "Failed to obtain time";
  }
  return timeStr;
}

bool isTimeToFeed(const char *timeStr) {
  int hour = (timeStr[0] - '0') * 10 + (timeStr[1] - '0');
  int minute = (timeStr[2] - '0') * 10 + (timeStr[3] - '0');
  int second = (timeStr[4] - '0') * 10 + (timeStr[5] - '0');
  bool isPM = strncmp(&timeStr[6], "PM", 2) == 0;


  if (isPM && hour != 12) hour += 12;
  if (!isPM && hour == 12) hour = 0;

  time_t now;
  time(&now);
  struct tm *timeInfo = localtime(&now);

  // Serial.print("projected: ");
  // Serial.print(hour);
  // Serial.print(minute);
  // Serial.print(second);
  // Serial.println(" ");

  // Serial.print("now: ");
  // Serial.print(timeInfo->tm_hour);
  // Serial.print(timeInfo->tm_min);
  // Serial.print(timeInfo->tm_sec);
  // Serial.println(" ");


  if (timeInfo->tm_hour == hour && timeInfo->tm_min == minute && timeInfo->tm_sec == second) {
    return true;
  }
  return false;
}

char *get_local_time(void) {
  time_t now;
  struct tm timeinfo;
  //char *formatted_time = (char *) malloc(10 * sizeof(char)); // Allocate memory for the formatted time string with an explicit cast
  //char *formatted_time = static_cast<char*>(malloc(10 * sizeof(char)));
  char *formatted_time = new char[10];
  // Get the current time
  time(&now);
  localtime_r(&now, &timeinfo);

  // Format the time according to the specified format
  strftime(formatted_time, 10, "%I%M%S%p", &timeinfo);

  return formatted_time;
}


void everyTwoCharacters(char *input, char *output) {
  int i = 0;
  int j = 0;

  while (input[i] != '\0') {
    output[j++] = input[i++];
    if (input[i] != '\0') {
      output[j++] = input[i++];
    }

    if (input[i] != '\0') {
      output[j++] = '-';
    }
  }

  output[j] = '\0';  // add null terminator to the output string
}

void printLocalTime() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.printf("Current time UTC: %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    Serial.println("Failed to obtain time");
  }
}

// current and voltage
// Function to map one range of values to another
float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to read the current multiple times, apply EMA filter and calculate the average
float readCurrentAverage() {
  static float filteredCurrent = 0;  // Filtered current value
  bool isCurrentDetected = false;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    int rawCurrent = analogRead(CURRENT_PIN);
    // Convert rawCurrent to actual current in mA
    float current = map((float)rawCurrent, 0, ADC_RESOLUTION, CURRENT_MIN, CURRENT_MAX);

    // Apply EMA filter to the current if it's greater than 0
    if (current > 0) {
      filteredCurrent = ALPHA * current + (1 - ALPHA) * filteredCurrent;
      isCurrentDetected = true;
    }

    delayMicroseconds(50);  // a small delay may be needed between reads
  }
  return isCurrentDetected ? filteredCurrent : 0;
}


void addToLogArray(int io_pin, int ma, int mv) {
  int state = (io_pin == sensor1) ? digitalRead(sensor1) : -1;  // read the pin state if it's the sensor1 pin

  if (io_pin == sensor1 && state == 1 ) {
    Serial.print("State of sensor1: ");
    Serial.println(state);
    delay(10000);
  }

  LogData data{ io_pin, ma, mv, state, "" };

  // Get the current time from NTP
  time_t now = time(nullptr);
  struct tm timeInfo;
  gmtime_r(&now, &timeInfo);

  // Format the timestamp as "YYYY-MM-DD HH:II:SS"
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
           timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
           timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);

  data.timestamp = buffer;

  logArray.push_back(data);
}

void deleteLogArray() {
  logArray.clear();
}

String json_log() {
  if (logArray.empty()) {
    return "";  // Return an empty string if the logArray is empty
  }

  DynamicJsonDocument doc(4096);  // Adjust the buffer size as needed

  for (const auto &data : logArray) {
    JsonObject entry = doc.createNestedObject();
    entry["io_pin"] = data.io_pin;
    entry["ma"] = data.ma;
    entry["mv"] = data.mv;
    entry["pin_state"] = data.pin_state;  // Add pin state to the json
    entry["timestamp"] = data.timestamp;
  }

  String jsonPacket;
  serializeJson(doc, jsonPacket);
  return jsonPacket;
}

void postJsonLogToServer(const String &jsonPacket) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // URL endpoint without the parameters
    String url = "https://mygrowbot.dwalliance.com/mygrowbot/post_logs.php";

    http.begin(url);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");  // Specify content-type header

    // Construct the POST data as key-value pairs
    String postData = "control_board_id=" + my_chip_id + "&json_log=" + jsonPacket;


    int httpResponseCode = http.POST(postData);  // Send the actual POST request with the constructed data

    if (httpResponseCode > 0) {

      String response = http.getString();  // Get the response to the request


      Serial.println(httpResponseCode);  // Print return code
      Serial.println(response);          // Print request answer
      Serial.println(postData);

    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();  // Close connection
  } else {
    Serial.println("Error in WiFi connection");
  }
}
