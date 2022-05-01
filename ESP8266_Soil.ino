/**
 * This sketch will use an Adafruit I2C soil sensor to show soil moisture and temperature levels.
 * The ESP8266/ESP-12E SCL pin is D1 (GPIO5), and SDA is D2 (GPIO4).
 * @copyright   Copyright © 2022 Adam Howell
 * @licence     The MIT License (MIT)
 */
#include <ESP8266WiFi.h>			// This header is part of the standard library.  https://www.arduino.cc/en/Reference/WiFi
#include <Wire.h>						// This header is part of the standard library.  https://www.arduino.cc/en/reference/wire
#include <PubSubClient.h>			// PubSub is the MQTT API.  Author: Nick O'Leary  https://github.com/knolleary/pubsubclient
#include "privateInfo.h"			// I use this file to hide my network information from random people browsing my GitHub repo.
#include "Adafruit_seesaw.h"		// Used to read the soil sensor.
#include <ArduinoJson.h>			// https://arduinojson.org/

/**
 * The commented-out variables are stored in "privateInfo.h", which I do not upload to GitHub.
 * If you do not want to use that file, you can set them here instead.
 */
//const char* wifiSsid = "your WiFi SSID";
//const char* wifiPassword = "your WiFi password";
//const char* mqttBroker = "your broker address";
//const int mqttPort = 1883;
const char* mqttTopic = "espWeather";								// The topic to publish to.
const char* commandTopic = "backYard/Lolin8266/command";		// This is a topic we subscribe to, to get updates.  Updates may change publishDelay or request an immediate poll of the sensors.
const char* sketchName = "ESP8266_Soil";
const char* notes = "Lolin ESP8266 with Adafruit I2C soil sensor";
const int LED_PIN = 2;											// This LED for the Lolin devkit is on the ESP8266 module itself (next to the antenna).
const int sdaGPIO = 5;
const int sclGPIO = 4;
char ipAddress[16];
char macAddress[18];
unsigned int loopCount = 0;									// This is a counter for how many loops have happened since power-on (or overflow).
unsigned long publishDelay = 60000;							// This is the delay between MQTT publishes.
unsigned long sensorPollDelay = 4000;						// This is the delay between polls of the soil sensor.  This should be greater than 100 milliseconds.
int mqttReconnectDelay = 5000;								// How long to wait (in milliseconds) between MQTT connection attempts.
unsigned long lastPublish = 0;								// This is used to determine the time since last MQTT publish.
unsigned long lastPoll = 0;									// This is used to determine the time since last sensor poll.
unsigned long bootTime;											// The time since boot.  This value "rolls" at about 50 days.
float soilTempC = 0.0;
uint16_t soilCapacitance = 0;


// Create class objects.
WiFiClient espClient;
PubSubClient mqttClient( espClient );
Adafruit_seesaw soilSensor;


void onReceiveCallback( char* topic, byte* payload, unsigned int length )
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
	str[i] = 0; // Null termination
	StaticJsonDocument <256> doc;
	deserializeJson( doc, str );

	// The command can be: publishTelemetry, changeTelemetryInterval, changeSeaLevelPressure, or publishStatus.
	const char* command = doc["command"];
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
		lastPublish = 0;
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


void readTelemetry()
{
	soilTempC = soilSensor.getTemp();
	soilCapacitance = soilSensor.touchRead( 0 );
}


void wifiConnect( int maxAttempts )
{
	// Announce WiFi parameters.
	String logString = "WiFi connecting to SSID: ";
	logString += wifiSsid;
	Serial.println( logString );

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
		char mqttString[512];
		snprintf( mqttString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\"\n}", sketchName, macAddress, ipAddress );
		mqttClient.publish( "espConnect", mqttString );
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

	wifiConnect( 20 );

	printUptime();
} // End of setup() function.


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


void publishTelemetry()
{
	// New format: <location>/<device>/<sensor>/<metric>
	/*
		backYard/Lolin8266/sketch
		backYard/Lolin8266/mac
		backYard/Lolin8266/ip
		backYard/Lolin8266/serial
		backYard/Lolin8266/rssi
		backYard/Lolin8266/loopCount
		backYard/Lolin8266/notes
		backYard/Lolin8266/seesawVersion
		backYard/Lolin8266/soil/tempC
		backYard/Lolin8266/soil/moisture
	*/
	// Print the signal strength:
	long rssi = WiFi.RSSI();
	Serial.print( "WiFi RSSI: " );
	Serial.println( rssi );
	// Prepare a String to hold the JSON.
	char mqttString[512];
	// Write the readings to the String in JSON format.
	snprintf( mqttString, 512, "{\n\t\"sketch\": \"%s\",\n\t\"mac\": \"%s\",\n\t\"ip\": \"%s\",\n\t\"tempC\": %.2f,\n\t\"moisture\": %d,\n\t\"rssi\": %ld,\n\t\"loopCount\": %d,\n\t\"notes\": \"%s\"\n}", sketchName, macAddress, ipAddress, soilTempC, soilCapacitance, rssi, loopCount, notes );
	// Publish the JSON to the MQTT broker.
	bool success = mqttClient.publish( mqttTopic, mqttString, false );
	if( success )
	{
		char buffer[20];
		// sketchName, macAddress, ipAddress, soilTempC, soilCapacitance, rssi, loopCount, notes
		mqttClient.publish( "backYard/Lolin8266/sketch", sketchName, false );
		mqttClient.publish( "backYard/Lolin8266/mac", macAddress, false );
		mqttClient.publish( "backYard/Lolin8266/ip", ipAddress, false );
		mqttClient.publish( "backYard/Lolin8266/rssi", ltoa( rssi, buffer, 10 ), false );
		mqttClient.publish( "backYard/Lolin8266/loopCount", ltoa( loopCount, buffer, 10 ), false );
		mqttClient.publish( "backYard/Lolin8266/notes", notes, false );
		dtostrf( soilTempC, 1, 3, buffer );
		mqttClient.publish( "backYard/Lolin8266/soil/tempC", buffer, false );
		dtostrf( soilCapacitance, 1, 3, buffer );
		mqttClient.publish( "backYard/Lolin8266/soil/moisture", buffer, false );

		Serial.println( "Successfully published this to the broker:" );
	}
	else
		Serial.println( "MQTT publish failed!  Attempted to publish this to the broker:" );
	// Print the JSON to the Serial port.
	Serial.println( mqttString );
	lastPublish = millis();
} // End of publishTelemetry() function.


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

	unsigned long time = millis();
	if( lastPoll == 0 || ( ( time > sensorPollDelay ) && ( time - sensorPollDelay ) > lastPoll ) )
	{
		readTelemetry();
		Serial.print( "Temperature: " );
		Serial.print( soilTempC );
		Serial.println("*C" );
		Serial.print( "Capacitance: " );
		Serial.println( soilCapacitance );
		Serial.println( "" );
		lastPoll = millis();
	}

	time = millis();
	// When time is less than publishDelay, subtracting publishDelay from time causes an overlow which results in a very large number.
	if( lastPublish == 0 || ( ( time > publishDelay ) && ( time - publishDelay ) > lastPublish ) )
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
