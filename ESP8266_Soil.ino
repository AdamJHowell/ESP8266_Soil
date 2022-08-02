/**
 * This sketch will use an Adafruit I2C soil sensor to show soil moisture and temperature levels.
 * The ESP8266/ESP-12E SCL pin is D1 (GPIO5), and SDA is D2 (GPIO4).
 * @copyright   Copyright © 2022 Adam Howell
 * @licence     The MIT License (MIT)
 *
 * Pseudocode:
 * 1. Read the soil moisture level (capacitance) every sensorPollDelay.
 * 2. Read the temperature from the soil moisture sensor.
 * 3. If moisture is less than minMoisture, run pump for pumpRunTime.
 * The pump is run in pumpRunTime millisecond increments instead of waiting for the reading to get above the minimum level.  Waiting could result in overwatering.
 * If the moisture level is still low after pumpRunTime millisectonds, it will trigger again during the next loop() after pumpMinOffDelay milliseconds has passed.
 *
 * Publish:
 * 	every time the pump runs: soilMoisture and soilTempC
 * 	every publishDelay: all stats (soilMoisture, soilTempC, loopCount, etc.)
 */
#include "ESP8266WiFi.h"			// This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#include <Wire.h>						// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include "privateInfo.h"			// This file holds my network credentials and addresses of servers.
#include "Adafruit_seesaw.h"		// Used to read the soil sensor. https://github.com/adafruit/Adafruit_Seesaw - Requires https://github.com/adafruit/Adafruit_BusIO
#include <ArduinoJson.h>			// https://arduinojson.org/

/**
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 * If you do not want to create that file, set them here instead.
 */
//const char* wifiSsid = "your WiFi SSID";
//const char* wifiPassword = "your WiFi password";
//const char* mqttBroker = "your broker address";
//const int mqttPort = 1883;
const char * sketchName = "ESP8266_Soil";
const char * notes = "Lolin ESP8266 with Adafruit I2C soil sensor";
const char * commandTopic = "backYard/Lolin8266/command";         // The topic used to subscribe to update commands.  Commands: publishTelemetry, changeTelemetryInterval, publishStatus.
const char * sketchTopic = "backYard/Lolin8266/sketch";           // The topic used to publish the sketch name.
const char * macTopic = "backYard/Lolin8266/mac";                 // The topic used to publish the MAC address.
const char * ipTopic = "backYard/Lolin8266/ip";                   // The topic used to publish the IP address.
const char * rssiTopic = "backYard/Lolin8266/rssi";               // The topic used to publish the WiFi Received Signal Strength Indicator.
const char * loopCountTopic = "backYard/Lolin8266/loopCount";     // The topic used to publish the loop count.
const char * notesTopic = "backYard/Lolin8266/notes";             // The topic used to publish notes relevant to this project.
const char * tempCTopic = "backYard/Lolin8266/soil/tempC";        // The topic used to publish the soil temperature.
const char * moistureTopic = "backYard/Lolin8266/soil/moisture";  // The topic used to publish the soil moisture.
const char * mqttTopic = "espWeather";                            // The topic used to publish a single JSON message containing all data.
char ipAddress[16];												// The IP address.
char macAddress[18];												// The MAC address to use as part of the MQTT client ID.
const unsigned int LED_PIN = 2;								// This LED for the Lolin devkit is on the ESP8266 module itself (next to the antenna).
const unsigned int sdaGPIO = 5;								// The GPIO to use for SDA.
const unsigned int sclGPIO = 4;								// The GPIO to use for SCL.
const unsigned int relayGPIO = 14;							// The GPIO which controls the relay.
unsigned int mqttReconnectDelay = 5000;					// How long to wait (in milliseconds) between MQTT connection attempts.
unsigned int loopCount = 0;									// This is a counter for how many loops have happened since power-on (or overflow).
unsigned long publishDelay = 60000;							// This is the delay between MQTT publishes.
unsigned long sensorPollDelay = 10000;						// This is the delay between polls of the soil sensor.  This should be greater than 100 milliseconds.
unsigned long pumpRunTime = 20000;							// Minimum time to run the pump.
unsigned long pumpMinOffDelay = 20000;						// The time to wait after stopping, before the pump will start again.  This allows water to flow through the soil.
unsigned long lastPublishTime = 0;							// This is used to determine the time since last MQTT publish.
unsigned long lastPollTime = 0;								// This is used to determine the time since last sensor poll.
unsigned long bootTime = 0;									// The time since boot.  This value "rolls" at about 50 days.
unsigned long pumpStartTime = 0;								// The most recent time that the pump started.
unsigned long pumpStopTime = 0;								// The most recent time that the pump stopped.
bool pumpRunning = false;										// Flag to indicate when the pump is running or not.
bool invalidTemp = false;										// Flag to indicate the last reading was out of boundaries.
bool invalidMoisture = false;									// Flag to indicate the last reading was out of boundaries.
float soilTempC = 0.0;											// The soil temperature in Celcius.
uint16_t soilMoisture = 0;										// The soil moisture level (capacitance).
uint16_t minMoisture = 500;									// The moisture level which triggers the pump.


// Create class objects.
WiFiClient espClient;
PubSubClient mqttClient( espClient );
Adafruit_seesaw soilSensor;


void onReceiveCallback( char * topic, byte * payload, unsigned int length )
{
	char str[length + 1];
	Serial.print( "Message arrived [" );
	Serial.print( topic );
	Serial.print( "] " );
	int i=0;
	for( i = 0; i < length; i++ )
	{
		Serial.print( ( char ) payload[i] );
		str[i] = ( char )payload[i];
	}
	Serial.println();
	// Add the null terminator.
	str[i] = 0;
	StaticJsonDocument <256> doc;
	deserializeJson( doc, str );

	// The command can be: publishTelemetry, changeTelemetryInterval, or publishStatus.
	const char * command = doc["command"];
	if( strcmp( command, "publishTelemetry") == 0 )
	{
		Serial.println( "Reading and publishing sensor values." );
		// Poll the sensor.
		readTelemetry();
		// Publish the sensor readings.
		publishTelemetry();
		Serial.println( "Readings have been published." );
	}
	else if( strcmp( command, "changeTelemetryInterval") == 0 )
	{
		Serial.println( "Changing the publish interval." );
		unsigned long tempValue = doc["value"];
		// Only update the value if it is greater than 4 seconds.  This prevents a seconds vs. milliseconds mixup.
		if( tempValue > 4000 )
			publishDelay = tempValue;
		Serial.print( "MQTT publish interval has been updated to " );
		Serial.println( publishDelay );
		lastPublishTime = 0;
	}
	else if( strcmp( command, "publishStatus") == 0 )
	{
		Serial.println( "publishStatus is not yet implemented." );
	}
	else
	{
		Serial.print( "Unknown command: " );
		Serial.println( command );
	}
} // End of onReceiveCallback() function.


/*
 * This function will reset the device.
 * It declares a reset function at address 0.
 * ToDo: Currently, this does not work.
 */
void(* resetFunc) (void) = 0;


/*
 * readTelemetry() will:
 * 1. read from all available sensors
 * 2. store legitimate values in global variables
 * 3. set a flag if any value is invalid
 */
void readTelemetry()
{
	float temporarySoilTempC = soilSensor.getTemp();
	uint16_t temporarySoilMoisture = soilSensor.touchRead( 0 );

	// Ignore obviously invalid temperature readings.
	if( temporarySoilTempC > -50 || temporarySoilTempC < 90 )
	{
		soilTempC = temporarySoilTempC;
		invalidTemp = false;
	}
	else
	{
		// If this is already true, meaning two polls in a row were bad, reboot the device.
		if( invalidTemp == true )
		{
			Serial.println("\n\n\n\nTwo consecutive bad values!\nResetting the device!\n\n");
			resetFunc();	// Call the reset function.
		}
		else
			invalidTemp = true;
	}

	// Ignore obviously invalid moisture readings.
	if( temporarySoilMoisture > 100 || temporarySoilMoisture < 1500 )
	{
		soilMoisture = temporarySoilMoisture;
		invalidMoisture = false;
	}
	else
	{
		// If this is already true, meaning two polls in a row were bad, reboot the device.
		if( invalidMoisture == true )
		{
			Serial.println("\n\n\n\nTwo consecutive bad values!\nResetting the device!\n\n");
			resetFunc();	// Call the reset function.
		}
		else
			invalidMoisture = true;
	}
}


/*
 * wifiConnect() will attempt to connect to the defined WiFi network up to maxAttempts times.
 */
void wifiConnect( int maxAttempts )
{
	// Announce WiFi parameters.
	Serial.print( "WiFi connecting to SSID: " );
	Serial.println( wifiSsid );

	// Connect to the WiFi network.
	Serial.printf( "Wi-Fi mode set to WIFI_STA %s\n", WiFi.mode( WIFI_STA ) ? "" : " - Failed!" );
	WiFi.begin( wifiSsid, wifiPassword );

	int i = 1;
	/*
     WiFi.status() return values:
     0 : WL_IDLE_STATUS when WiFi is in process of changing between statuses
     1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
     3 : WL_CONNECTED after successful connection is established
     4 : WL_CONNECT_FAILED if wifiPassword is incorrect
     6 : WL_DISCONNECTED if module is not configured in station mode
  */
	// Loop until WiFi has connected.
	while( WiFi.status() != WL_CONNECTED && i < maxAttempts )
	{
		digitalWrite( LED_PIN, 1 ); // Turn the LED off.
		delay( 1000 );
		Serial.println( "Waiting for a connection..." );
		Serial.print( "WiFi status: " );
		Serial.println( WiFi.status() );
		Serial.print( i++ );
		Serial.println( " seconds" );
	}

	if( WiFi.status() == WL_CONNECTED )
	{
		WiFi.setAutoReconnect( true );
		WiFi.persistent( true );

		// Print that WiFi has connected.
		Serial.println( '\n' );
		Serial.println( "WiFi connection established!" );
		Serial.print( "MAC address: " );
		Serial.println( macAddress );
		Serial.print( "IP address: " );
		snprintf( ipAddress, 16, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
		Serial.println( ipAddress );

		digitalWrite( LED_PIN, 0 );	// Turn the WiFi LED on.
	}
} // End of wifiConnect() function.


// mqttConnect() will attempt to (re)connect the MQTT client.
bool mqttConnect( int maxAttempts )
{
	digitalWrite( LED_PIN, 1 ); // Turn the LED off.
	Serial.print( "Attempting to connect to the MQTT broker up to " );
	Serial.print( maxAttempts );
	Serial.println( " times." );

	int i = 0;
	// Loop until MQTT has connected.
	while( !mqttClient.connected() && i < maxAttempts )
	{
		Serial.print( "Attempt # " );
		Serial.print( i + 1 );
		Serial.print( "..." );

		char clientId[22];
		// Put the macAddress and a random number into clientId.  The random number suffix prevents brokers from rejecting a clientID as already in use.
		snprintf( clientId, 22, "%s-%03d", macAddress, random( 999 ) );
		Serial.print( "Connecting with client ID '" );
		Serial.print( clientId );
		Serial.print( "' " );

		// Connect to the broker using the pseudo-random clientId.
		if( mqttClient.connect( clientId ) )
		{
			Serial.println( " connected!" );
			digitalWrite( LED_PIN, 0 ); // Turn the LED on.
		}
		else
		{
			Serial.print( " failed!  Return code: " );
			Serial.print( mqttClient.state() );
			Serial.print( ".  Trying again in " );
			Serial.print( mqttReconnectDelay / 1000 );
			Serial.println( " seconds." );
			digitalWrite( LED_PIN, 0 ); // Turn the LED on.
			delay( mqttReconnectDelay / 2 );
			digitalWrite( LED_PIN, 1 ); // Turn the LED off.
			delay( mqttReconnectDelay / 2 );
		}
		i++;
	}
	if( mqttClient.connected() )
	{
		// Subscribe to backYard/Lolin8266/command, which will respond to publishTelemetry and publishStatus
		mqttClient.subscribe( commandTopic );
		mqttClient.setBufferSize( 512 );
		char connectString[512];
		snprintf( connectString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\"\n}", sketchName, macAddress, ipAddress );
		mqttClient.publish( "espConnect", connectString );
		digitalWrite( LED_PIN, 0 ); // Turn the LED on.
	}
	else
	{
		Serial.println( "Unable to connect to the MQTT broker!" );
		return false;
	}

	Serial.println( "Function mqttConnect() has completed." );
	return true;
} // End of mqttConnect() function.


/*
 * setup() will initialize the device and connected components.
 */
void setup()
{
	delay( 500 );								// A pause to give me time to open the serial monitor.
	pinMode( LED_PIN, OUTPUT );			// Initialize digital pin WiFi LED as an output.
	digitalWrite( LED_PIN, 0 );			// Turn the LED on.
	Wire.begin( sdaGPIO, sclGPIO );		// Initialize I2C communication.

	Serial.begin( 115200 );
	if( !Serial )
		delay( 1000 );

	Serial.println( '\n' );
	Serial.print( sketchName );
	Serial.println( " is beginning its setup()." );
	Serial.println( __FILE__ );

	pinMode( relayGPIO, OUTPUT );			// Set the replay (pump) GPIO as an output.
	digitalWrite( relayGPIO, LOW );		// Turn the relay (pump) off.

	// Set the ipAddress char array to a default value.
	snprintf( ipAddress, 16, "127.0.0.1" );

	// Set the MQTT client parameters.
	mqttClient.setServer( mqttBroker, mqttPort );
	mqttClient.setCallback( onReceiveCallback );				 // Assign the onReceiveCallback() function to handle MQTT callbacks.

	Serial.println( "Attempting to connect to the soil sensor..." );
	if( !soilSensor.begin( 0x36 ) )
		Serial.println( "\n\nERROR: soil sensor not found!\n\n" );
	else
	{
		Serial.print( "Seesaw started! version: " );
		Serial.println( soilSensor.getVersion(), HEX );
	}

	// Set the MAC address variable to its value.
	snprintf( macAddress, 18, "%s", WiFi.macAddress().c_str() );
	Serial.print( "MAC address: " );
	Serial.println( macAddress );

	wifiConnect( 5 );

	printUptime();
} // End of setup() function.


/*
 * printUptime() will print the uptime to the serial port.
 */
void printUptime()
{
	Serial.print( "Uptime in " );
	long seconds = ( millis() - bootTime ) / 1000;
	long minutes = seconds / 60;
	long hours = minutes / 60;
	if( seconds < 601 )
	{
		Serial.print( "seconds: " );
		Serial.println( seconds );
	}
	else if( minutes < 121 )
	{
		Serial.print( "minutes: " );
		Serial.println( minutes );
	}
	else
	{
		Serial.print( "hours: " );
		Serial.println( hours );
	}
} // End of printUptime() function.


/*
 * publishTelemetry() will publish the sensor and device data over MQTT.
 */
void publishTelemetry()
{
	// Print the signal strength:
	long rssi = WiFi.RSSI();
	Serial.print( "WiFi RSSI: " );
	Serial.println( rssi );
	// Prepare a String to hold the JSON.
	char mqttString[512];
	// Write the readings to the String in JSON format.
	snprintf( mqttString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"tempC\": %.2f,\n\t\"moisture\": %d,\n\t\"rssi\": %ld,\n\t\"loopCount\": %d,\n\t\"notes\": \"%s\"\n}", sketchName, macAddress, ipAddress, soilTempC, soilMoisture, rssi, loopCount, notes );
	// Publish the JSON to the MQTT broker.
	bool success = mqttClient.publish( mqttTopic, mqttString, false );
	if( success )
	{
		Serial.println( "Succsefully published to:" );
		char buffer[20];
		// New format: <location>/<device>/<sensor>/<metric>
		if( mqttClient.publish( sketchTopic, sketchName, false ) )
			Serial.println( sketchTopic );
		if( mqttClient.publish( macTopic, macAddress, false ) )
			Serial.println( macTopic );
		if( mqttClient.publish( ipTopic, ipAddress, false ) )
			Serial.println( ipTopic );
		if( mqttClient.publish( rssiTopic, ltoa( rssi, buffer, 10 ), false ) )
			Serial.println( rssiTopic );
		if( mqttClient.publish( loopCountTopic, ltoa( loopCount, buffer, 10 ), false ) )
			Serial.println( loopCountTopic );
		if( mqttClient.publish( notesTopic, notes, false ) )
			Serial.println( notesTopic );
		dtostrf( soilTempC, 1, 3, buffer );
		if( mqttClient.publish( tempCTopic, buffer, false ) )
			Serial.println( tempCTopic );
		if( mqttClient.publish( moistureTopic, ltoa( soilMoisture, buffer, 10 ), false ) )
			Serial.println( moistureTopic );

		Serial.print( "Successfully published to '" );
		Serial.print( mqttTopic );
		Serial.println( "', this JSON:" );
	}
	else
		Serial.println( "MQTT publish failed!  Attempted to publish this JSON to the broker:" );
	// Print the JSON to the Serial port.
	Serial.println( mqttString );
	lastPublishTime = millis();
} // End of publishTelemetry() function.


/*
 * runPump() will turn the pump on and off as needed.
 *
 * The pump will turn on when:
 * 	The pump is not currently running.
 * 	The moisture level is low.
 * 	It has been off for at least pumpMinOffDelay milliseconds.
 *
 * The pump will turn off when:
 * 	The pump is currently running.
 * 	The pump has been running for at least pumpRunTime milliseconds.
 * 	Note that it does not check the moisture level.  This is deliberate.
 */
void runPump()
{
	unsigned long currentTime = millis();

	// If the soil is dry and the pump is not running.
	if( !pumpRunning && ( soilMoisture < minMoisture ) )
	{
		// If enough time has passed since the pump was shut off (so to give time for water to flow to the sensor).
		if( currentTime - pumpStopTime > pumpMinOffDelay )
		{
			Serial.println( "Starting the pump." );
			// Note the start time.
			pumpStartTime = currentTime;
			// Turn the relay (pump) on.
			digitalWrite( relayGPIO, HIGH );
			// Flag that the pump is now running.
			pumpRunning = true;
		}
	}

	// If the pump is currently running.
	if( pumpRunning )
	{
		// If enough time has passed since the pump was started.
		if( currentTime - pumpStartTime > pumpRunTime )
		{
			Serial.println( "Stopping the pump." );
			// Note the start time.
			pumpStopTime = currentTime;
			// Turn the relay (pump) off.
			digitalWrite( relayGPIO, LOW );
			// Flag that the pump has stopped.
			pumpRunning = false;
		}
	}
} // End of runPump() function.


void loop()
{
	// Reconnect to WiFi if necessary.
	if( WiFi.status() != WL_CONNECTED )
		wifiConnect( 10 );
	// Check the mqttClient connection state.
	if( !mqttClient.connected() )
		mqttConnect( 10 );
	// The loop() function facilitates the receiving of messages and maintains the connection to the broker.
	mqttClient.loop();

	// Check if the pump should be run.
	runPump();

	unsigned long time = millis();
	if( lastPollTime == 0 || ( ( time > sensorPollDelay ) && ( time - sensorPollDelay ) > lastPollTime ) )
	{
		readTelemetry();
		Serial.print( "Temperature: " );
		Serial.print( soilTempC );
		Serial.println(" C" );
		Serial.print( "Moisture: " );
		Serial.println( soilMoisture );
		Serial.println( "" );
		lastPollTime = millis();
	}

	time = millis();
	// When time is less than publishDelay, subtracting publishDelay from time causes an overlow which results in a very large number.
	if( lastPublishTime == 0 || ( ( time > publishDelay ) && ( time - publishDelay ) > lastPublishTime ) )
	{
		loopCount++;
		// These next 3 lines act as a "heartbeat", to give local users a visual indication that the system is working.
		digitalWrite( LED_PIN, 1 );	// Turn the WiFi LED off to alert the user that a publish is about to take place.
		delay( 1000 );						// Wait for one second.
		digitalWrite( LED_PIN, 0 );	// Turn the WiFi LED on.

		Serial.println( sketchName );
		Serial.print( "Connected to broker at \"" );
		Serial.print( mqttBroker );
		Serial.print( ":" );
		Serial.print( mqttPort );
		Serial.println( "\"" );
		Serial.print( "Listening for command messages on topic \"" );
		Serial.print( commandTopic );
		Serial.println( "\"." );
		Serial.print( "Seesaw started! version: " );
		Serial.println( soilSensor.getVersion(), HEX );

		printUptime();
		publishTelemetry();

	  	Serial.print( "Next publish in " );
		Serial.print( publishDelay / 1000 );
		Serial.println( " seconds.\n" );
	}
} // End of loop() function.
