/*     
    tv-cover-control 
    Bernhard Frenking
*/
const char* software_version = "v1.4";

#include "secrets.h"
#include <ESP8266WiFi.h> 
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>

// pins of wemos D1 mini (ESP8266)
#define PIN_STEP_R      D0
#define PIN_STEP_L      D6
#define PIN_DIR         D5
#define PIN_SENSOR_R    D2
#define PIN_SENSOR_L    A0 // analog pin configured as digital input
#define PIN_TV_PWR      D7
#define PIN_TVPC_PWR    D3
#define PIN_PWR         D1
#define PIN_BRK         D8
//#define PIN_ENA        // enable stepper driver - if floating, always on for stepper motor driver ISD04

// positions as steps
#define CLOSE_STEPS 0               
#define TV_STEPS 15125             
#define OPEN_STEPS 17000      
#define PERCENT 170      // for moving in percent of travel (hard coded to prevent calc. during runtime)

// stepper and break tuning
#define STEP_PULS_DURATION 5    // low level >4µs for stepper motor driver ISD04
#define STEP_DURATION 450       // defines speed of stepper motor in µs 
#define PWR_DELAY 700           // delay for power in ms
#define BRK_DELAY 700           // delay for break in ms

// wifi
#define WIFI_CONNECT_TIMEOUT_S 15

// HTTP commands and webserver
const char* host = "TV-Cover-Control";
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

// MQTT
const char* tv_position_topic                   = "tv/cover/set";         
const char* tv_position_state_topic             = "tv/cover/state";  
const char* tv_availability_state_topic         = "tv/cover/availability";
const char* tv_position_percentage_topic        = "tv/cover/set_position";
const char* tv_position_percentage_state_topic  = "tv/cover/position";
const char* tv_pwr_switch_topic                 = "tv/switchTv/set";
const char* tv_pwr_switch_state_topic           = "tv/switchTv/state";
const char* tvpc_pwr_switch_topic               = "tvpc/power/set";
const char* tvpc_pwr_switch_state_topic         = "tvpc/power/state";
const char* tv_brake_switch_topic               = "tv/switchBrake/set";
const char* tv_brake_switch_state_topic         = "tv/switchBrake/state";

String tmp_str; // String for publishing MQTT messages
char buf[5]; // buffer publishing MQTT messages

// requests/flags
bool stop_req = false;
bool moving   = false;
bool homing   = true;
bool contactLeft;
bool contactRight;

// step position
int current_position  = CLOSE_STEPS;
int target_position   = CLOSE_STEPS;
unsigned long t       = 0;

// step side for left and/or right side (stepper motor) defined 
// based on homing sensor contact state
enum side {
  left,
  right,
  either,
  both
};
side stepSide;  
side getStepSide();

// wifi init
long lastReconnectAttempt;
WiFiClient espClient;

// mqtt init
PubSubClient client(espClient);

// Setup ------------------------------------------------------------------------------------------------
void setup() {
  // Sets the pins
  pinMode(PIN_SENSOR_R, INPUT_PULLUP);  // Initialize Sensor
  pinMode(PIN_SENSOR_L, INPUT);         // Initialize Sensor
  pinMode(PIN_STEP_R, OUTPUT);
  pinMode(PIN_STEP_L, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_TV_PWR, OUTPUT);
  pinMode(PIN_TVPC_PWR, OUTPUT);
  pinMode(PIN_PWR, OUTPUT);
  pinMode(PIN_BRK, OUTPUT);

  // preset pins
  digitalWrite(PIN_TV_PWR,  HIGH);  // disable TV PWR
  digitalWrite(PIN_TV_PWR,  HIGH);  // disable TVPC PWR
  digitalWrite(PIN_PWR,     HIGH);  // disable 24V PWR
  digitalWrite(PIN_BRK,     HIGH);  // fasten break
  digitalWrite(PIN_DIR,     HIGH);  // up
  
  //connect to WiFi
  WiFi.mode(WIFI_STA);
  setup_wifi();
  
  // httpServer
  MDNS.begin(host);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  httpServer.on("/", handleRoot);
  httpServer.on("/tvPcOn", handle_tvPcOn);
  httpServer.on("/tvPcOff", handle_tvPcOff);  
  httpServer.on("/tvOn", handle_tvOn);
  httpServer.on("/tvOff", handle_tvOff);
  httpServer.on("/posDown", handle_posDown);
  httpServer.on("/posUp", handle_posUp);    
  httpServer.on("/down1", handle_down1);
  httpServer.on("/up1", handle_up1);  
  httpServer.on("/down10", handle_down10);
  httpServer.on("/up10", handle_up10);
  httpServer.on("/posOpen", handle_posOpen);
  httpServer.on("/posTv", handle_posTv);
  httpServer.on("/posClose", handle_posClose);
  httpServer.on("/stop", handle_stop);
  httpServer.on("/homing", handle_homing);
  httpServer.on("/brakeOn", handle_brakeOn);
  httpServer.on("/brakeOff", handle_brakeOff);
  httpServer.on("/24VOn", handle_24VOn);
  httpServer.on("/24VOff", handle_24VOff);
  httpServer.on("/restart", handle_restart);
  httpServer.on("/pre_update", handle_update);
  httpServer.onNotFound(handle_NotFound);

  // MQTT
  client.setServer(SECRET_MQTT_HOST, 1883);
  client.setCallback(callback);
  if (!client.connected()) {
    connectMQTT(); // Wifi and MQTT
  }
  
  // publish MQTT status messages
  publish_tv_availability_state_topic("online");
  publish_tvPcPwrSwitch_state_topic("OFF");  
  publish_tvPwrSwitch_state_topic("OFF");
  publish_brakeSwitch_state_topic("OFF");

  // start serial
  //Serial.begin(9600);
  //Serial.println("Setup done");
  
} // setup()

void loop() {
  
  // maintain wifi and fetch httpUpdate
  lastReconnectAttempt = checkWifi(lastReconnectAttempt); 
  
  // maintain MQTT connection
  if ( !client.loop() ) { // false, if not connected
    connectMQTT(); // Wifi and MQTT
    // publish MQTT status messages
    publish_tv_availability_state_topic("online");
    //publish_tvPwrSwitch_state_topic(switchRequest);
    //publish_brakeSwitch_state_topic(switchRequest);
  }  

  if (current_position > OPEN_STEPS || current_position < 0) {
    stop_req == true;
  }

  if (homing == true) {
    // fetch sensor contact states
    contactLeft  = checkSensorContact(left);
    contactRight = checkSensorContact(right);
    if ( contactLeft && contactRight ) { // homing done
      current_position = CLOSE_STEPS;
      target_position = CLOSE_STEPS;
      homing = false;
    } else {
      if (moving == false) { 
        current_position = 1;
        target_position = 0;
        moving = movePrep(current_position, target_position); 
      }      
      if (stop_req == false) {                            // move one step
        step(getStepSide(contactLeft, contactRight));
      } else {                                            // stop, if requested     
        target_position = current_position;
        stop_req = false;     
        homing = false;
      } 
    }
    
    // on target position fasten break and shut off stepper
    if (target_position == current_position) {
      moving = onTarget(current_position); // and set moving to false      
    } // on target     
  
  } // homing

  // move
  else if(current_position != target_position) {
        
    // move up
    if ( (current_position > target_position) ) { 

      // fetch sensor contact states
      contactLeft  = checkSensorContact(left);
      contactRight = checkSensorContact(right);  
      
      if ( (contactLeft || contactRight) ) { // close position
        current_position = CLOSE_STEPS;
        target_position = CLOSE_STEPS; 
      } 
      else {                                             
        if (stop_req == false) {            // move one step
          // enable stepper with direction and release break on first loop of movement
          if (moving == false) { 
            //Serial.println("moving up");
            moving = movePrep(current_position, target_position);
          }          
          step(getStepSide(contactLeft, contactRight));
          current_position--;
        } else if (stop_req == true) {     // stop, if requested
          stop_req = false;          
          target_position = current_position;
        }
      }
    } // move up
    
    // move down
    else { 
      if (stop_req == false) {
        // enable stepper with direction and release break on first loop of movement
        if (moving == false) {
          //Serial.println("moving up"); 
          moving = movePrep(current_position, target_position);
        }        
        step(both);
        current_position++;        
      } else if (stop_req == true) {
        stop_req = false;
        target_position = current_position;
      }
    } 

    // on target position fasten break and shut off stepper
    if (target_position == current_position) {
      moving = onTarget(current_position); // and set moving to false        
    } // on target

  } // move
  
} // loop

// functions ------------------------------------------------------------------------------------------------

bool onTarget(int current_position) {
  digitalWrite(PIN_BRK, HIGH); // fasten break
  publish_tv_position_percentage_state_topic(current_position);      
  delay(BRK_DELAY);
  digitalWrite(PIN_PWR, HIGH); // disable 24V for stepper
  switch(current_position) {
    case CLOSE_STEPS:
      publish_tv_position_state_topic("closed"); // open, opening, closed, closing
      break;
    default:
      publish_tv_position_state_topic("open"); // open, opening, closed, closing
      break;
  }  
  return false; // moving
}

bool movePrep(int current_position, int target_position) {
    if ( current_position > target_position )  { 
    digitalWrite(PIN_DIR, HIGH); // up
    publish_tv_position_state_topic("closing"); // open, opening, closed, closing
    //Serial.println("move first loop: set dir to up");
    //Serial.print("current position: ");
    //Serial.println(current_position);
    //Serial.print("target position: ");
    //Serial.println(target_position);
  } else {
    digitalWrite(PIN_DIR, LOW); // down        
    publish_tv_position_state_topic("opening"); // open, opening, closed, closing
    //Serial.println("move first loop: set dir to down");
    //Serial.print("current position: ");
    //Serial.println(current_position);
    //Serial.print("target position: ");
    //Serial.println(target_position);        
  }
  digitalWrite(PIN_PWR, LOW); // enable 24V for stepper
  delay(PWR_DELAY);
  digitalWrite(PIN_BRK, LOW); // release break
  delay(BRK_DELAY);
  return true; // moving
}

side getStepSide(bool contactLeft, bool contactRight) {
  if (!contactLeft && !contactRight) {
    return both;
  } else if (contactLeft && !contactRight) {
    return right;
  } else if (!contactLeft && contactRight) {
    return left;
  }
}

void publish_tv_availability_state_topic(String availability_state) {
  availability_state.toCharArray(buf, availability_state.length() + 1);
  client.publish(tv_availability_state_topic, buf, true); 
}

void publish_tv_position_state_topic(String position_state) {
  position_state.toCharArray(buf, position_state.length() + 1);
  client.publish(tv_position_state_topic, buf, true);
}

void publish_tv_position_percentage_state_topic(int current_position) {
  tmp_str = String(current_position * (float)100 / OPEN_STEPS); //converting to percent and string
  tmp_str.toCharArray(buf, tmp_str.length() + 1);
  client.publish(tv_position_percentage_state_topic, buf, true);
}

void publish_tvPwrSwitch_state_topic(String switch_tv) {
  switch_tv.toCharArray(buf, switch_tv.length() + 1);
  client.publish(tv_pwr_switch_state_topic, buf);
}

void publish_tvPcPwrSwitch_state_topic(String switch_tvPcPwr) {
  switch_tvPcPwr.toCharArray(buf, switch_tvPcPwr.length() + 1);
  client.publish(tvpc_pwr_switch_state_topic, buf);
}

void publish_brakeSwitch_state_topic(String switch_brake) {
  switch_brake.toCharArray(buf, switch_brake.length() + 1);
  client.publish(tv_brake_switch_state_topic, buf);
}

bool checkSensorContact(side choosenSide) {
  switch(choosenSide) {
    case either:
      if (digitalRead(PIN_SENSOR_R) == HIGH || analogRead(PIN_SENSOR_L) > 150) {
        //Serial.println("Sensor contact on one or either sides");
        return true;
      } else {
        return false;
      }
      break;
    case left:
      if (analogRead(PIN_SENSOR_L) > 150) {
        //Serial.println("Sensor contact left");
        return true;
      } else {
        return false;
      }
      break;    
    case right:
      if (digitalRead(PIN_SENSOR_R)) {
        //Serial.println("Sensor contact right");
        return true;
      } else {
        return false;
      }
      break;    
  }
}

void step(side stepSide) {
  yield();  

  switch (stepSide) {
    case both:
      digitalWrite(PIN_STEP_R, LOW);
      digitalWrite(PIN_STEP_L, LOW);
      break;
    case left:
      digitalWrite(PIN_STEP_L, LOW);
      break;
    case right:
      digitalWrite(PIN_STEP_R, LOW);
      break;
  } 
  
  delayMicroseconds(STEP_PULS_DURATION);
  
  switch (stepSide) {
    case both:
      digitalWrite(PIN_STEP_R, HIGH);
      digitalWrite(PIN_STEP_L, HIGH);
      break;
    case left:
      digitalWrite(PIN_STEP_L, HIGH);
      break;
    case right:
      digitalWrite(PIN_STEP_R, HIGH);
      break;
  }   
  
  // wait until STEP_DURATION is reached - for consistent step times
  unsigned long dt = micros() - t;
  if (dt < STEP_DURATION) {
    delayMicroseconds(STEP_DURATION - dt);
  }
  t = micros();
  
} // step

void switchTv(String switchRequest) {
  if (switchRequest == "OFF") {
    digitalWrite(PIN_TV_PWR, HIGH); // disable 
  } else if (switchRequest == "ON"){
    digitalWrite(PIN_TV_PWR, LOW); // enable 
  }
  publish_tvPwrSwitch_state_topic(switchRequest);
} 

void switchTvPc(String switchRequest) {
  if (switchRequest == "OFF") {
    digitalWrite(PIN_TVPC_PWR, HIGH); // disable 
  } else if (switchRequest == "ON"){
    digitalWrite(PIN_TVPC_PWR, LOW); // enable 
  }
  publish_tvPcPwrSwitch_state_topic(switchRequest);
} 

void switchBrake(String switchRequest) {
  if (switchRequest == "OFF") {
    digitalWrite(PIN_BRK, HIGH); // disable 
    delay(BRK_DELAY);
    digitalWrite(PIN_PWR, HIGH); // disable
  } else if (switchRequest == "ON"){
    digitalWrite(PIN_PWR, LOW); // enable 
    delay(PWR_DELAY);
    digitalWrite(PIN_BRK, LOW); // enable    
  }
  publish_brakeSwitch_state_topic(switchRequest);
} 

void switch24V(String switchRequest) {
  if (switchRequest == "OFF") {
    digitalWrite(PIN_PWR, HIGH); // disable 
  } else if (switchRequest == "ON"){
    digitalWrite(PIN_PWR, LOW); // enable
  }
} 
  
void up1() {
  if (current_position >= PERCENT)
  target_position = current_position - PERCENT;
}

void down1() {
  if (current_position < (OPEN_STEPS - PERCENT))
    target_position = current_position + PERCENT;  
}

void up10() {
  if (current_position >= PERCENT*10)
  target_position = current_position - PERCENT*10;
}

void down10() {
  if (current_position < (OPEN_STEPS - PERCENT*10))
    target_position = current_position + PERCENT*10;  
}

void posUp() {
  if (current_position > CLOSE_STEPS) {
    if (current_position <= TV_STEPS ) {
      target_position = CLOSE_STEPS;  
    }
    else if (current_position <= OPEN_STEPS ) {
      target_position = TV_STEPS;  
    }
  }  
}

void posClose() {
  target_position = CLOSE_STEPS;  
}

void posTv() {
  target_position = TV_STEPS;  
}

void posOpen() {
  target_position = OPEN_STEPS;  
}

void posDown() {
  if (current_position < OPEN_STEPS) {
    if (current_position >= TV_STEPS ) {
      target_position = OPEN_STEPS;  
    }
    else if (current_position >= CLOSE_STEPS ) {
      target_position = TV_STEPS;  
    }
  }    
}

void stop() {
  stop_req = true;
}

void restart() {
  ESP.restart();
}

long checkWifi(long lastReconnectAttempt) {
  if (WiFi.status() == WL_CONNECTED) {    // check if WiFi connection is present
    // Wifi is connected
    httpServer.handleClient(); // for HTTPupdate
  }
  else { // Client is not connected
    long now = millis();
    if (now - lastReconnectAttempt > 10000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      setup_wifi();
      if (WiFi.status() == WL_CONNECTED)
        lastReconnectAttempt = 0;
    }
  }
  return lastReconnectAttempt;
} // checkWifi()

void setup_wifi() {
  delay(10);
  //Serial.print("Wifi setup"); 
  //WiFi.config(ip, gateway, subnet, dns1); 
  WiFi.begin(SECRET_SSID, SECRET_PSW);
  uint32_t time1 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    //Serial.print(".");
    //if (millis() > time1 + (WIFI_CONNECT_TIMEOUT_S * 1000))
  }
  //Serial.println("");
  //Serial.println("WiFi connected");
  //Serial.println("IP address: ");
  //Serial.println(WiFi.localIP());
} // setup_wifi()

void callback(char* topic, byte * payload, unsigned int length) { 
  String topicString = (char*)topic;
  String payloadCmdAsString = (char*)payload;
  payload[length] = '\0'; // Add end of line at end of char array to mark is as a string    
  String position_req = (char *)payload;

  // only execute if not moving or pause or reset commands
  if (moving == false || (moving == true && ( position_req == "stop" || position_req == "restart" ))) {
    // topic position in percent
    if (topicString == tv_position_percentage_topic) {
      String temp_s = payloadCmdAsString.substring(0, length);
      float temp_f = (float)temp_s.toInt(); // toInt returns long  
      target_position = (int)((temp_f / 100) * (float)OPEN_STEPS); // from percentage to STEPS
    }
    
    // topic tv power switch
    else if (topicString == tv_pwr_switch_topic) {
      switchTv((char *)payload);
    }

    // topic tvpc power switch
    else if (topicString == tvpc_pwr_switch_topic) {
      switchTvPc((char *)payload);
    }
  
    // topic tv elevator brake switch
    else if (topicString == tv_brake_switch_topic) {
      switchBrake((char *)payload);
    }
  
    // topic position 
    else if (topicString == tv_position_topic) {
      if (position_req == "close") {
        posClose();
      }
      else if (position_req == "tv") {
        posTv();
      }
      else if (position_req == "open") {
        posOpen();
      }
      else if (position_req == "homing") {
        homing = true;
      }    
      else if (position_req == "down1") {
        down1();
      }        
      else if (position_req == "up1") {
        up1();
      }   
      else if (position_req == "down10") {
        down10();
      }        
      else if (position_req == "up10") {
        up10();
      }   
      else if (position_req == "posDown") {
        posDown();
      }        
      else if (position_req == "posUp") {
        posUp();
      }       
      else if (position_req == "stop") {
        stop();
      }           
      else if (position_req == "restart") {
        restart();
      } else {
      }              
    } // tv_position_topic
  }
} // mqtt callback

// connect mqtt
boolean connectMQTT() {
  //Serial.println("reconnect");
  if (WiFi.status() != WL_CONNECTED) {    // check if WiFi connection is present
    //Serial.println("reconnect setup_wifi");
    setup_wifi();
  }
  if (client.connect(SECRET_MQTT_ID, SECRET_MQTT_USER, SECRET_MQTT_PSW)) {
    //Serial.println("reconnect mqtt");
    // ... and resubscribe
    client.subscribe(tv_pwr_switch_topic,           1); // qos = 1
    client.subscribe(tvpc_pwr_switch_topic,         1); // qos = 1
    client.subscribe(tv_brake_switch_topic,         0); // qos = 0
    client.subscribe(tv_position_percentage_topic,  0); // qos = 0
    client.subscribe(tv_position_topic,             0); // qos = 0
  }
  //Serial.print("MQTT connected: ");
  //Serial.println(client.connected());
  return client.connected();
} // connectMQTT()

void handleRoot() {
  //Serial.println("Connected to client");
  httpServer.send(200, "text/html", SendHTML());
}
void handle_OnConnect() {
  //Serial.println("Connected to client");
  httpServer.send(200, "text/html", SendHTML());
}

void handle_tvOn() {
  switchTv("ON");
  httpServer.send(200, "text/html", SendHTML());
}
void handle_tvOff() {
  switchTv("OFF");
  httpServer.send(200, "text/html", SendHTML());
}

void handle_tvPcOn() {
  switchTvPc("ON");
  httpServer.send(200, "text/html", SendHTML());
}
void handle_tvPcOff() {
  switchTvPc("OFF");
  httpServer.send(200, "text/html", SendHTML());
}

void handle_brakeOn() {
  switchBrake("ON");
  httpServer.send(200, "text/html", SendHTML());
}
void handle_brakeOff() {
  switchBrake("OFF");
  httpServer.send(200, "text/html", SendHTML());
}

void handle_24VOn() {
  switch24V("ON");
  httpServer.send(200, "text/html", SendHTML());
}
void handle_24VOff() {
  switch24V("OFF");
  httpServer.send(200, "text/html", SendHTML());
}

void handle_posUp() {
  posUp();
  httpServer.send(200, "text/html", SendHTML());
}
void handle_posDown() {
  posDown();
  httpServer.send(200, "text/html", SendHTML());
}

void handle_up1() {
  up1();
  httpServer.send(200, "text/html", SendHTML());
}
void handle_down1() {
  down1();
  httpServer.send(200, "text/html", SendHTML());
}

void handle_up10() {
  up10();
  httpServer.send(200, "text/html", SendHTML());
}
void handle_down10() {
  down10();
  httpServer.send(200, "text/html", SendHTML());
}

void handle_posOpen() {
  target_position = OPEN_STEPS;
  httpServer.send(200, "text/html", SendHTML());
}
void handle_posTv() {
  target_position = TV_STEPS;
  httpServer.send(200, "text/html", SendHTML());
}
void handle_posClose() {
  target_position = CLOSE_STEPS;
  httpServer.send(200, "text/html", SendHTML());
}

void handle_stop() {
  stop(); 
  httpServer.send(200, "text/html", SendHTML());
}

void handle_homing() {
  homing = true;
  httpServer.send(200, "text/html", SendHTML());
}

void handle_restart() {
  restart();
  httpServer.send(200, "text/html", SendHTML());
}

void handle_update() {
  //target_position = TV_STEPS;;
  httpServer.send(200, "text/html", "<a href=\"/update\">Update</a>");
}

void handle_NotFound() {
  httpServer.send(404, "text/plain", "Not found");
}

String SendHTML() {
  String ptr = "<!DOCTYPE html><html>\n";
  ptr += "<head><meta name=\"viewport\"content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr += "<title>";
  // change line below to change the name of the webpage 
  ptr += host;
  ptr += "</title>\n";
  ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr += "body{margin-top: 50px;} h1 {color: #444444;margin: 25px auto 30px;} h3 {color: #444444;margin-bottom: 30px;}\n";
  ptr += ".button {width: 150px;background-color: #1abc9c;border: none;color: white;padding: 13px 10px;text-decoration: none;font-size: 20px;margin: 0px auto 15px;cursor: pointer;border-radius: 4px;}\n";
  ptr += ".button-0 {background-color: #eb3434;}\n";
  ptr += ".button-0:active {background-color: #2c3e50;}\n";
  ptr += ".button-10 {background-color: #00ff00;}\n";
  ptr += ".button-10:active {background-color: #00ff00;}\n";
  ptr += ".button-1 {background-color: #34495e;}\n";
  ptr += ".button-1:active {background-color: #2c3e50;}\n";
  ptr += ".button-2 {background-color: #0d41d1;}\n";
  ptr += ".button-2:active {background-color: #082985;}\n";
  ptr += ".button-update {background-color: #a32267;}\n";
  ptr += ".button-update:active {background-color: #961f5f;}\n";
  ptr += "p {font-size: 18px;color: #383535;margin-bottom: 15px;}\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";

  ptr += "<h1>TV-Controller</h1>\n";
  ptr += software_version;

  ptr += "<p>TV-Pwr</p><a class="" href=""></a>\n";
  ptr += "<a class=\"button button-10\" href=\"/tvOn\">On</a>&nbsp;\n";
  ptr += "<a class=\"button button-0\" href=\"/tvOff\">Off</a><br><br><br>\n";

  ptr += "<p>TVPC-Pwr</p><a class="" href=""></a>\n";
  ptr += "<a class=\"button button-10\" href=\"/tvPcOn\">On</a>&nbsp;\n";
  ptr += "<a class=\"button button-0\" href=\"/tvPcOff\">Off</a><br><br><br>\n";
  
  ptr += "<p>24V/Stepper on and brake release </p><a class="" href=""></a>\n";
  ptr += "<a class=\"button button-10\" href=\"/brakeOn\">On</a>&nbsp;\n";
  ptr += "<a class=\"button button-0\" href=\"/brakeOff\">Off</a><br><br><br>\n";

  ptr += "<p>24V/Stepper on </p><a class="" href=""></a>\n";
  ptr += "<a class=\"button button-10\" href=\"/24VOn\">On</a>&nbsp;\n";
  ptr += "<a class=\"button button-0\" href=\"/24VOff\">Off</a><br><br><br>\n";
  
  ptr += "<p>Control picture position</p><a class="" href=""></a>\n";
  ptr += "<a class=\"button button-1\" href=\"/posClose\">Close</a>&nbsp;\n";
  ptr += "<a class=\"button button-1\" href=\"/posTv\">TV</a>&nbsp;\n";
  ptr += "<a class=\"button button-1\" href=\"/posOpen\">Open</a><br><br><br>\n";

  ptr += "<a class=\"button button-2\" href=\"/posUp\">Pos up</a>&nbsp;\n";
  ptr += "<a class=\"button button-2\" href=\"/posDown\">Pos down</a><br><br><br>\n";

  ptr += "<a class=\"button button-2\" href=\"/up1\">Up 1%</a>&nbsp;\n";
  ptr += "<a class=\"button button-2\" href=\"/down1\">Down 1%</a><br><br><br>\n";

  ptr += "<a class=\"button button-2\" href=\"/up10\">Up 10%</a>&nbsp;\n";
  ptr += "<a class=\"button button-2\" href=\"/down10\">Down 10%</a><br><br><br>\n";

  ptr += "<a class=\"button button-update\" href=\"/homing\">Homing</a>\n";
  ptr += "<a class=\"button button-0\" href=\"/stop\">Stop</a><br><br><br>\n";

  ptr += "<p>ESP8266</p><a class=\"button button-update\" href=\"/restart\">Restart</a>\n";

  ptr += "<a class=\"button button-update\" href=\"/pre_update\">Update</a>\n";

  ptr += "</body>\n";
  ptr += "</html>\n";
  return ptr;
}
