#include <WiFi.h>
#include "Esp32MQTTClient.h"
#include <WiFiManager.h> 
#include "driver/pcnt.h"   //Pulse counter library
#include <ArduinoJson.h>
#include <ezTime.h>     

//// PULSE COUNTER MODULE ////
#define PCNT_FREQ_UNIT      PCNT_UNIT_0     // select ESP32 pulse counter unit 0 (out of 0 to 7 indipendent counting units)
                                            // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html
int16_t PulseCounter =     0;                                // pulse counter, max. value is 65536
int OverflowCounter =      0;                                // pulse counter overflow counter
int PCNT_H_LIM_VAL =       30000;                            // upper limit of counting  max. 32767, write +1 to overflow counter, when reached 
uint16_t PCNT_FILTER_VAL=  0;                             // filter (damping, inertia) value for avoiding glitches in the count, max. 1023
pcnt_isr_handle_t user_isr_handle = NULL;                 // user interrupt handler (not used)
int liters = 0;                                           // number of liters that pass through the flux sensor
int old_liters = 0;
#define PULSES_PER_LITER 165                              // 165 pulses from the flux sensor tell that a liter has passed

//// Sanitizer level pumps status////
volatile int P1_status = 0;                               //status of the sanitizer pump 1 (0: OFF, 1: ON)
volatile int P2_status = 0;                               //status of the sanitizer pump 2 (0: OFF, 1: ON)
volatile int P1_pulses = 0;                               //number of impulses to send to the sanitizer pump 1
volatile int P2_pulses = 0;                               //number of impulses to send to the sanitizer pump 2
volatile int p1pulsescounter = 0;                         //counter used to perform requested impulses on pump 1
volatile int p2pulsescounter = 0;                         //counter used to perform requested impulses on pump 2
volatile int p1toggle = 1;                                //variable used for toggling pump 1 pin
volatile int p2toggle = 1;                                //variable used for toggling pump 2 pin
//// Chlorine sensor reading variable ////
float chlorine_concentration = 0.1;                       // concentration of chlorine given by crs1 (range: 0.1 ppm - 20 ppm)
// Level sensor status
int SL1_status;                                           // 0: low level, 1: high level
int old_SL1_status;
//// firmware version of the device and device id ////
#define SW_VERSION "0.1"
#define DEVICE_ID "geniale board 1"     
//// Other handy variables ////
volatile bool new_request = false;                        // flag that tells if a new request has arrived from the hub
volatile int received_msg_id = 0;                         // used for ack mechanism
volatile int received_msg_type = -1;                      // if 0 the device is sending its status
                                                          // if 1 the HUB wants to change the status of the device (with the values passed in the message)
                                                          // if 2 the device ACKs the HUB in response to a command
// defines for message type 
#define STATUS 0
#define SET_VALUES 1
#define ACK_HUB 2

// STATUS LED HANDLING
#define LED_CHANNEL 0
#define RESOLUTION 8
#define LED_PWM_FREQ 10
#define OFF 0
#define BLINK_5HZ 128
#define ON 255

////  MICROSOFT AZURE IOT DEFINITIONS   ////
static const char* connectionString = "HostName=geniale-iothub.azure-devices.net;DeviceId=00000001;SharedAccessKey=Cn4UylzZVDZD8UGzCTJazR3A9lRLnB+CbK6NkHxCIMk=";
static bool hasIoTHub = false;
static bool hasWifi = false;
#define INTERVAL 10000               // IoT message sending interval in ms
#define MESSAGE_MAX_LEN 256
int messageCount = 1;                // tells the number of the sent message
//static bool messageSending = true;
//static uint64_t send_interval_ms;

////  I/Os definitions    ////
#define SL1_GPIO  35                 // Level sensor connected to GPIO35
#define PCNT_INPUT_SIG_IO   34       // Flow sensor connected to GPIO34
#define P2_GPIO   33                 // Sanitizer pump P2 contact connected to GPIO33
#define P1_GPIO   32                 // Sanitizer pump P1 contact connected to GPIO32
#define LED   5                      // Status led connected to GPIO5

// Create a timer to generate an ISR at a defined frequency in order to sample the system
hw_timer_t * timer = NULL;
#define OVF_MS 1000                      // The timer interrupt fires every second
volatile bool new_status = false;       // When it's true a sensor has changed its value and it needs to be sent

void IRAM_ATTR onTimer(){            // Timer ISR, called on timer overflow every OVF_MS
  // Read status of sensors  //
  get_liters();
  SL1_status = ( digitalRead(SL1_GPIO) == 0 ) ? 1 : 0;

  if( liters != old_liters || SL1_status != old_SL1_status ) 
    new_status = true;

  if(P1_status == 1) {
    if(p1pulsescounter < 2*P1_pulses) {
      digitalWrite(P1_GPIO, p1toggle);
      p1toggle = !p1toggle;                       
      p1pulsescounter++; 
    }     
    else {
        P1_status = 0;
        p1pulsescounter = 0;
        new_status = true;    // update p1 status (bring it back to 0)
    }   
  }
  if(P2_status == 1) {
    if(p2pulsescounter < 2*P2_pulses) {
      digitalWrite(P2_GPIO, p2toggle); 
      p2toggle = !p2toggle;       
      p2pulsescounter++;                      
      }
    else {
      P2_status = 0;
      p2pulsescounter = 0;
      new_status = true;    // update p2 status (bring it back to 0)    
    }
  }  
  old_liters = liters;
  old_SL1_status = SL1_status;
}

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    //Serial.println("Send Confirmation Callback finished.");
  }
}

static void MessageCallback(const char* payLoad, int size)
{
  ledcWrite(LED_CHANNEL, ON);
  Serial.println("Received message from HUB");
  if (size < 256) { 
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payLoad);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    }
    else {  
    new_request = true;
    received_msg_id = doc["message_id"];
    received_msg_type = doc["message_type"];
      if(received_msg_type == SET_VALUES) {
          P1_status = doc["P1"];
          P1_pulses = doc["P1_pulses"];
          P2_status = doc["P2"];
          P2_pulses = doc["P2_pulses"];
      }
    }
  }
  else Serial.println("Cannot parse message, too long!");
}

/* NOT USED - DEVICE TWIN CALLBACK
static void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payLoad, int size)
{
  char *temp = (char *)malloc(size + 1);
  if (temp == NULL)
  {
    return;
  }
  memcpy(temp, payLoad, size);
  temp[size] = '\0';
  // Display Twin message.
  Serial.println(temp);
  free(temp);
}
*/
/* NOT USED - DEVICE METHOD CALLBACK
  static int  DeviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *response_size)
  {
    LogInfo("Try to invoke method %s", methodName);
    const char *responseMessage = "\"Successfully invoke device method\"";
    int result = 200;

    if (strcmp(methodName, "start") == 0)
    {
      LogInfo("Start sending temperature and humidity data");
      messageSending = true;
    }
    else if (strcmp(methodName, "stop") == 0)
    {
      LogInfo("Stop sending temperature and humidity data");
      messageSending = false;
    }
    else
    {
      LogInfo("No method %s found", methodName);
      responseMessage = "\"No method found\"";
      result = 404;
    }

    *response_size = strlen(responseMessage) + 1;
    *response = (unsigned char *)strdup(responseMessage);

    return result;
  }
*/

//// PULSE COUNTER OVERFLOW ISR ////
  void IRAM_ATTR CounterOverflow(void *arg) {                  // Interrupt for overflow of pulse counter
    OverflowCounter = OverflowCounter + 1;                     // increase overflow counter
    PCNT.int_clr.val = BIT(PCNT_FREQ_UNIT);                    // clear overflow flag
    pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit
  }

  void initPulseCounter (){                                    // initialise pulse counter
    pcnt_config_t pcntFreqConfig = { };                        // Instance of pulse counter
    pcntFreqConfig.pulse_gpio_num = PCNT_INPUT_SIG_IO;         // pin assignment for pulse counter
    pcntFreqConfig.pos_mode = PCNT_COUNT_INC;                  // count rising edges (=change from low to high logical level) as pulses
    pcntFreqConfig.neg_mode = PCNT_COUNT_DIS;                  // do nothing on falling edges
    pcntFreqConfig.counter_h_lim = PCNT_H_LIM_VAL;             // set upper limit of counting 
    pcntFreqConfig.unit = PCNT_FREQ_UNIT;                      // select ESP32 pulse counter unit 0
    pcntFreqConfig.channel = PCNT_CHANNEL_0;                   // select channel 0 of pulse counter unit 0
    pcnt_unit_config(&pcntFreqConfig);                         // configure rigisters of the pulse counter
  
    pcnt_counter_pause(PCNT_FREQ_UNIT);                        // pause pulse counter unit
    pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit
  
    pcnt_event_enable(PCNT_FREQ_UNIT, PCNT_EVT_H_LIM);         // enable event for interrupt on reaching upper limit of counting
    pcnt_isr_register(CounterOverflow, NULL, 0, &user_isr_handle);  // configure register overflow interrupt handler
    pcnt_intr_enable(PCNT_FREQ_UNIT);                          // enable overflow interrupt

    pcnt_set_filter_value(PCNT_FREQ_UNIT, PCNT_FILTER_VAL);    // set damping, inertia 
    pcnt_filter_enable(PCNT_FREQ_UNIT);                        // enable counter glitch filter (damping)
  
    pcnt_counter_resume(PCNT_FREQ_UNIT);                       // resume counting on pulse counter unit
  }
   
  void Reset_PCNT() {                                          // function resetting counter 
    OverflowCounter = 0;                                       // set overflow counter to zero
    pcnt_counter_clear(PCNT_FREQ_UNIT);                        // zero and reset of pulse counter unit
  }

  void get_liters(){                                    // converts the pulses received from fl1 to liters
      pcnt_get_counter_value(PCNT_FREQ_UNIT, &PulseCounter);     // get pulse counter value - maximum value is 16 bit
      liters = ( OverflowCounter*PCNT_H_LIM_VAL + PulseCounter ) / PULSES_PER_LITER;
  }

void setup() {
  pinMode(PCNT_INPUT_SIG_IO, INPUT);                            // the output of the flow sensor is open collector (MUST USE EXTERNAL PULL UP!!)
  pinMode(SL1_GPIO, INPUT);
  pinMode(P1_GPIO, OUTPUT);     
  pinMode(P2_GPIO, OUTPUT);
  digitalWrite(P1_GPIO, LOW);                                   // P1 and P2 are initially off
  digitalWrite(P2_GPIO, LOW);
  SL1_status = ( digitalRead(SL1_GPIO) == 0 ) ? 1 : 0;          // read the status of the level sensor
  // configure LED PWM functionalitites
  ledcSetup(LED_CHANNEL, LED_PWM_FREQ, RESOLUTION);
  ledcAttachPin(LED, LED_CHANNEL);                              // Attach PWM module to status LED
  ledcWrite(LED_CHANNEL, BLINK_5HZ);                            // LED initially blinks at 5Hz
  initPulseCounter();
  Serial.begin(115200);
  Serial.println("ESP32 Device");
  Serial.println("Initializing...");
  Serial.println(" > WiFi");
  Serial.println("Starting connecting WiFi.");

  delay(10);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  WiFiManager wm;
  //wm.resetSettings();  // reset settings - wipe stored credentials for testing
  bool res;
  res = wm.autoConnect("GENIALE brd1 setup"); // Generates a pwd-free ap for the user to connect and tell Wi-Fi credentials
  //res = wm.autoConnect("AutoConnectAP","password"); // Generates a pwd-protected ap for the user to connect and tell Wi-Fi credentials
  if(!res) {
      Serial.println("Failed to connect to wifi");
      delay(10000);
      ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("Connected to wifi!");
      ledcWrite(LED_CHANNEL, ON);
      // Wait for ezTime to get its time synchronized
	    waitForSync();
      Serial.println("UTC Time in ISO8601: " + UTC.dateTime(ISO8601));
      hasWifi = true;
    }
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(" > IoT Hub");
  if (!Esp32MQTTClient_Init((const uint8_t*)connectionString, true))
  {
    hasIoTHub = false;
    Serial.println("Initializing IoT hub failed.");
    return;
  }
  hasIoTHub = true;
  Esp32MQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
  Esp32MQTTClient_SetMessageCallback(MessageCallback);
  //Esp32MQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
  //Esp32MQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);
  
  randomSeed(analogRead(0));
  //send_interval_ms = millis();
  /* Use 1st timer of 4 */
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  timer = timerBegin(0, 80, true);
  /* Attach onTimer function to our timer */
  timerAttachInterrupt(timer, &onTimer, true);
  /* Set alarm to call onTimer function every OVF_MS milliseconds. 
  1 tick is 1us*/
  /* Repeat the alarm (third parameter) */
  timerAlarmWrite(timer, 1000*OVF_MS, true);
  /* Start an alarm */
  timerAlarmEnable(timer);
  Serial.println("ISR Timer started");
  ledcWrite(LED_CHANNEL, OFF);
  Serial.println("Waiting for messages from HUB...");
}

void send_message(int reply_type, int msgid) {
if (hasWifi && hasIoTHub)
  {
      StaticJsonDocument<256> msgtosend;            // pre-allocate 256 bytes of memory for the json message
      msgtosend["message_id"] = msgid;
      msgtosend["timestamp"] = UTC.dateTime(ISO8601);
      msgtosend["message_type"] = reply_type;
      msgtosend["device_id"] = DEVICE_ID;
      msgtosend["iot_module_software_version"] = SW_VERSION;
      msgtosend["SL1"] = SL1_status;                // Closed contact means LOW sanitizer level!
      msgtosend["CRS1"] = chlorine_concentration;
      msgtosend["FL1"] = liters;
      msgtosend["P1"] = P1_status;
      msgtosend["P2"] = P2_status;
      char out[256];
      int msgsize =serializeJson(msgtosend, out);
      //Serial.println(msgsize);
      Serial.println("Sending message to HUB:");
      Serial.println(out);
      EVENT_INSTANCE* message = Esp32MQTTClient_Event_Generate(out, MESSAGE);
      Esp32MQTTClient_SendEventInstance(message);
      ledcWrite(LED_CHANNEL, OFF);
  }
}


void loop() {
  Esp32MQTTClient_Check();
  // if a request has arrived from the hub, process it and send a reply
  if(new_request == true){
    new_request = false;
    switch (received_msg_type)  {
      case SET_VALUES: 
        send_message(ACK_HUB, received_msg_id);
        break;
      case STATUS:
        send_message(STATUS, received_msg_id);
        break;
      default:
        Serial.println("Invalid message type!");
        ledcWrite(LED_CHANNEL, OFF);
        break;
    }
  }
  if(new_status == true) {
    new_status = false;
    send_message(STATUS, messageCount);
    messageCount++;
  }
}