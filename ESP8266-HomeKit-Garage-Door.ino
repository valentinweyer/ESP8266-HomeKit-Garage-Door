#include <Arduino.h>
#include <arduino_homekit_server.h>
#include "wifi_info.h"

#include <Adafruit_Sensor.h>


#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);

#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN          0
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED        1
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING       2
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING       3
//#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_STOPPED       4
#define HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_UNKNOWN       255

#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN           0
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED         1
#define HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN        255

/*#define HOMEKIT_CHARACTERISTIC_LOCK_CURRENT_STATE_UNSECURED 0
#define HOMEKIT_CHARACTERISTIC_LOCK_CURRENT_STATE_SECURED 1
#define HOMEKIT_CHARACTERISTIC_LOCK_CURRENT_STATE_JAMMED 2
#define HOMEKIT_CHARACTERISTIC_LOCK_CURRENT_STATE_UNKNOWN 3

#define HOMEKIT_CHARACTERISTIC_LOCK_TARGET_STATE_UNSECURED 0
#define HOMEKIT_CHARACTERISTIC_LOCK_TARGET_STATE_SECURED 1
#define HOMEKIT_CHARACTERISTIC_LOCK_TARGET_STATE_JAMMED 2
#define HOMEKIT_CHARACTERISTIC_LOCK_TARGET_STATE_UNKNOWN 3*/


// Set GPIOs for LED and reedswitch
const int openSensorPin = 4;
const int closedSensorPin = 5;
const int PIN_OPERATOR_CONTROL = 14;


// Auxiliary variables (it will only detec`t changes that are 1500 milliseconds apart)
unsigned long previousMillis = 0; 
const long interval = 1500;

// Interrupts cannot be stored in flash; tell the compiler to put it in RAM
void ICACHE_RAM_ATTR handle_sensor_change(); 

// When one of the pins changes, toggle a variable to TRUE so we can respond inside the main loop()
bool sensor_interrupt = FALSE;
void handle_sensor_change() 
{
  sensor_interrupt = TRUE;
}

// access your homekit characteristics defined in my_accessory.c
extern "C" homekit_server_config_t config;
extern "C" homekit_characteristic_t cha_current_door_state;
extern "C" homekit_characteristic_t cha_target_door_state;
extern "C" homekit_characteristic_t cha_obstruction_detected;
//extern "C" homekit_characteristic_t cha_name;
//extern "C" homekit_characteristic_t cha_lock_current_state;
//extern "C" homekit_characteristic_t cha_lock_target_state;


static uint32_t next_heap_millis = 0;
static uint32_t next_report_millis = 0;


// Called when the value is read by iOS Home APP
homekit_value_t cha_programmable_switch_event_getter() 
{
	// Should always return "null" for reading, see HAP section 9.75
	return HOMEKIT_NULL_CPP();
}


// Called when setting target door state
void cha_target_door_state_setter(const homekit_value_t value) 
{
  // State value requested by HomeKit
  cha_target_door_state.value = value;
	LOG_D("Target door state: %i", value.uint8_value);

  // If the current state is not equal to the target state, then we "push the button"; otherwise, we do nothing
  //if (cha_current_door_state.value.uint8_value != cha_target_door_state.value.uint8_value) 

  if(cha_current_door_state.value.uint8_value == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN || cha_current_door_state.value.uint8_value == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED) 
  {
    Serial.println(cha_current_door_state.value.uint8_value);
    Serial.println("Travel initiated");
    digitalWrite(PIN_OPERATOR_CONTROL, HIGH);
    delay(500);
    digitalWrite(PIN_OPERATOR_CONTROL, LOW);
  }
  else 
  {
    //Reverse the travelling state
    Serial.println("Reversing Travel");
    if(cha_current_door_state.value.uint8_value == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING)
    {
      cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING;
    }
    else if(cha_current_door_state.value.uint8_value == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING)
    {
      cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING;
    }

    Serial.println("Reverse travel");
    Serial.println(cha_current_door_state.value.uint8_value);
    digitalWrite(PIN_OPERATOR_CONTROL, HIGH);
    delay(500);
    digitalWrite(PIN_OPERATOR_CONTROL, LOW);
    delay(500);
    digitalWrite(PIN_OPERATOR_CONTROL, HIGH);
    delay(500);
    digitalWrite(PIN_OPERATOR_CONTROL, LOW);
  } 
}


void setup() 
{
	Serial.begin(115200);  

  // Read the current door state
  pinMode(openSensorPin, INPUT_PULLUP);
  pinMode(closedSensorPin, INPUT_PULLUP);
  pinMode(PIN_OPERATOR_CONTROL, OUTPUT);
  digitalWrite(PIN_OPERATOR_CONTROL, LOW);

	wifi_connect(); // in wifi_info.h
	my_homekit_setup();

  cha_target_door_state.setter = cha_target_door_state_setter;

  // Set interrupts to watch for changes in the open/close sensors
  attachInterrupt(digitalPinToInterrupt(closedSensorPin), handle_sensor_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(openSensorPin), handle_sensor_change, CHANGE);

  // Initialize the current door state
  homekit_value_t current_state = cha_current_door_state_getter();
  homekit_characteristic_notify(&cha_current_door_state, cha_current_door_state.value);

  // Initialize target door state based on the current door state
  switch (cha_current_door_state.value.uint8_value) 
  {
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED: 
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED; 
      break;
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING: 
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED; 
      break;
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN: 
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN; 
      break;
    case HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING: 
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN; 
      break;
    default: 
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_UNKNOWN; 
      break;
  }  
  homekit_characteristic_notify(&cha_target_door_state, cha_target_door_state.value);

}

void loop() 
{
  // If a change in one of the sensors has been detected, re-run the current state getter
  if (sensor_interrupt == TRUE) 
  {
    homekit_value_t new_state = cha_current_door_state_getter();
    LOG_D("Current state: %.1f", new_state);
    homekit_characteristic_notify(&cha_current_door_state, new_state);
    sensor_interrupt = FALSE;
  }

  my_homekit_loop();

	delay(10);
}


void my_homekit_setup() 
{

  // Set the setters and getters
	cha_current_door_state.getter = cha_current_door_state_getter;
	cha_target_door_state.setter = cha_target_door_state_setter;
	//cha_lock_current_state.getter = cha_lock_current_state_getter;
	//cha_obstruction_detected.getter = cha_obstruction_detected_getter;

  arduino_homekit_setup(&config);
}

void my_homekit_loop() 
{
	arduino_homekit_loop();
	const uint32_t t = millis();
	if (t > next_report_millis) 
  {
		// report sensor values every 10 seconds
		next_report_millis = t + 1 * 1000;
		my_homekit_report();
	}
	if (t > next_heap_millis) 
  {
		// show heap info every 5 seconds
		next_heap_millis = t + 5 * 1000;
		LOG_D("Free heap: %d, HomeKit clients: %d",
				ESP.getFreeHeap(), arduino_homekit_connected_clients_count());
	}
}

void my_homekit_report() 
{
	//cha_temperature.value.float_value = temperature_value;
	//LOG_D("Current temperature: %.1f", temperature_value);
	//homekit_characteristic_notify(&cha_temperature, cha_temperature.value);
}


// Called when getting current door state
homekit_value_t cha_current_door_state_getter() 
{
  Serial.print("OpenSensor: ");
  Serial.println(digitalRead(openSensorPin));
  Serial.print("ClosedSensor: ");
  Serial.println(digitalRead(closedSensorPin));
  
  // Stash the current state so we can detect a change
  homekit_characteristic_t current_state = cha_current_door_state;

  // Read the sensors and use some logic to determine state
  if (digitalRead(openSensorPin) == HIGH) 
  {
    cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN;
    Serial.println("OPEN");
  } 
  else if(digitalRead(closedSensorPin) == HIGH) 
  {
    cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED;
    Serial.println("CLOSED");
  } 
  else if(digitalRead(openSensorPin) == LOW && digitalRead(closedSensorPin) == LOW)
  {
    Serial.println("Neither closed nor open");
    // If neither, then the door is in between switches, so we use the last known state to determine which way it's probably going
    if(current_state.value.uint8_value == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSED) 
    {
      // Current door state was "closed" so we are probably now "opening"
      cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING;
      Serial.println("OPENING");
    } 
    else if(current_state.value.uint8_value == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPEN) 
    {
      // Current door state was "opened" so we are probably now "closing"
      cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING;
      Serial.println("CLOSING");
    }

    // If it is traveling, then it might have been started by the button in the garage. Set the new target state:
    if (cha_current_door_state.value.uint8_value == HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_OPENING) 
    {
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_OPEN;
    } 
    else if(cha_current_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_CURRENT_DOOR_STATE_CLOSING) 
    {
      cha_target_door_state.value.uint8_value = HOMEKIT_CHARACTERISTIC_TARGET_DOOR_STATE_CLOSED;
    }
    // ... and then notify HomeKit clients
  	LOG_D("Target door state: %i", cha_target_door_state.value.uint8_value);
    homekit_characteristic_notify(&cha_target_door_state, cha_target_door_state.value);
  }

	LOG_D("Current door state: %i", cha_current_door_state.value.uint8_value);
	return cha_current_door_state.value;
}