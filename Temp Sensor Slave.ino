/*
 Name:		Temp_Sensor_Slave.ino
 Created:	4/19/2019 7:15:58 PM
 Author:	Robbie
*/

/*	I will need to remove the extra code to be the master using this code once I figure out how I want to handle
the data on the master side with the TFT screen.
*/

#include <OneWire.h>
#include <SPI.h>
#include <RF24.h>
#include <DallasTemperature.h>		//Dallas Temperature library

//  PIN MAPPING
#define TEMP_SENSOR_PIN 2
#define ADDR_PIN_0 14	//	A0
#define ADDR_PIN_1 15	//	A1
#define ADDR_PIN_2 16	//	A2

//  TEMP NODE SETTINGS
byte nodeAddress;
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

//  NRF2401 RADIO SETTINGS
RF24 radio(10, 9); // CE, CSN
const byte addresses[][6] = { "00001", "00002" };

//	COMMAND PACKET STRUCT **MAX 32 BYTES**
struct Command_Package {
	byte com_address;	//	intended address for the command
	byte command;		//	the command to be issued
	};
Command_Package comm;	//	create a variable for the above structure

//  DATA PACKAGE STRUCT **MAX 32 BYTES**
struct Data_Package {
	byte origin_addr;	//	originating address
	byte temp_1I;		//	1 bytes
	byte temp_1D;		//	1 bytes
	byte temp_2I;		//	1 bytes
	byte temp_2D;		//	1 bytes
};
Data_Package data;		//	create a variable for the above structure

//  INITIALIZE THE DS18B20 TEMPERATURE SENSORS
//  ***********************************************
//	Set the pin that the temp sensors are connected to
#define TEMP_SENSOR_PRECISION 11		//	set the temperature precision to 11 bits.  this is needed to get the proper precision for 1 decimal
OneWire oneWire(TEMP_SENSOR_PIN);			//  Setup oneWire instances to communicate with any OneWire Devices
DallasTemperature tempSensors(&oneWire);	//	pass onewire reference to Dallas temperatures.
DeviceAddress tempSensorAddr[4];			//  arrays to hold device addresses for 4 sensors
byte numberOfSensors = 0;	//	define the byte to store the number of sensors that is found on the pin
byte tempType = 1;			//  initializes the byte tempType
byte tempPrecision;			//	initializes the byte tempPrecision
float tempRead[4];			//	array to hold the temperature readings taken
char* tempSensorNameGraph[] = { "Temp 1", "Temp 2", "Temp 3", "Temp 4", "Temp 5" };		//	names for the graphic display

void setup() {
	//  SETUP THE SERIAL PORT
	Serial.begin(115200);			//  start the serial port

	//	SETUP THE PINS
	pinMode(TEMP_SENSOR_PIN, INPUT);
	pinMode(ADDR_PIN_0, INPUT);
	pinMode(ADDR_PIN_1, INPUT);
	pinMode(ADDR_PIN_2, INPUT);

	//	SETUP THE ADDRESS OF THIS NODE
	nodeAddress = FindMyAddress();

	//  SETUP THE DS18B20 SENSORS
	//	unknown why, but i have to add these again to get it working correctly
	new OneWire(TEMP_SENSOR_PIN);		//	setup a new instance of OneWire
	new DallasTemperature(&oneWire);	//	setup a new instance of DallasTemperature
	delay(500);							//	small delay to allow it to get the number of sensors
	tempSensors.begin();				//	start the DallasTemperature library
	FindTempSensors();					//	find all of the temp sensors connected Max = 4

	//	SETUP THE NRF24LO1+ RADIO
	//  ***********************************************
	radio.begin();

	if (nodeAddress == 0) {
		radio.openWritingPipe(addresses[0]);	// "00002"	TX
		radio.openReadingPipe(1, addresses[1]); // "00001"	RX
		Serial.println("Master Set");
	}
	else {
		radio.openWritingPipe(addresses[1]);	// "00001"	TX
		radio.openReadingPipe(1, addresses[0]); // "00002"	RX
		Serial.println("Slave Set");
	}
	radio.setPALevel(RF24_PA_MIN);

	//	START THE LOOP
	Serial.println();
	Serial.printf("Starting Loop :%u\n", millis());

}
void loop() {
	int radioread;

	//	Master
	if (nodeAddress == 0) {

		//	Request the temp reading from the remote station address 1
		for (byte i = 0; i < 3; i++)
		{
			comm.com_address = 1 + i;
			comm.command = 1;
			//	Send the command
			radio.stopListening();
			radio.write(&comm, sizeof(Command_Package));
			Serial.printf("Command %d Sent to Address %d\n", comm.command, comm.com_address);

			radio.startListening();
			Serial.print("Waiting\n");

			// Check whether we keep trying to recieve data or we timeout
			lastReceiveTime = millis();				// set the time for a timeout comparison

			while (!radio.available()) {
				//	if it has been more than second since we started listening break from the while loop
				currentTime = millis();
				if (currentTime - lastReceiveTime > 700) {
					Serial.print("Timout Occured\n\n");
					break;
				}
			}

			if (radio.available()) {
				radio.read(&data, sizeof(Data_Package));
				Serial.print("Recieved\n");
				Serial.printf("Add = %d, T1 = %d.%d: T2 = %d.%d\n\n", data.origin_addr, data.temp_1I, data.temp_1D, data.temp_2I, data.temp_2D);
			}
		}
		radio.stopListening();
	}

	//	Slaves
	else {
		radio.startListening();
		while (!radio.available());
		radio.read(&comm, sizeof(Command_Package));
		Serial.printf("Command %d RXd for Address %d\n", comm.command, comm.com_address);

		if (comm.com_address == nodeAddress) {

			if (comm.command == 1) {
				radio.stopListening();
				ReportTempSensors();

				radio.write(&data, sizeof(Data_Package));
				Serial.println("data sent");
				Serial.println();
			}
		}
	}
}

void ReportTempSensors()
//  Read the DS18B20 sensors that are attached to the oneWire bus on the TEMP_SENSOR_PIN and print to the appropriate device
{
	//	have all the sensors on the bus start a temperature conversion
	tempSensors.requestTemperatures();

	//	cycle through each one of the sensors
	for (uint8_t j = 0; j < 2; j++) {   // loop through the number of sensors found
		String addrString;				//	string to print the address
		uint8_t sensorConnected = 0;	//	int to compare if the current sensor is connected and readable
		int i = 0;	//	integer to store the integer portion of the sensor reading
		int d = 0;	//	integer to store the decimal portion of the sensor reading
		int nodec = 0;

		//	convert the address of the sensor to a string
		addrString = convertTempSensorAddress(tempSensorAddr[j]);

		// determine if there is a sensor connected to the current address
		if (tempSensors.isConnected(tempSensorAddr[j]) == true) {
			sensorConnected = 1;	//	see if there is a sensor at the given address
			if (tempType == 0) tempRead[j] = tempSensors.getTempC(tempSensorAddr[j]);	//	request the temp data from the sensor in C
			else tempRead[j] = tempSensors.getTempF(tempSensorAddr[j]);					//	request the temp data from the sensor in F
		}
		else sensorConnected = 0;

		//	convert the float to 2 integer parts.  this is necessary because the mega board cannot handle a printf with a float
		i = int(tempRead[j]);		//	get the integer portion of the reading
		d = round((tempRead[j] - i) * 10);	//	round the decimal portion of the reading and make it an integer
		if (d == 10) { d = 1; }				//	prevent the overflow of the decimal if it was equal to 1 before the int conversion

		data.origin_addr = nodeAddress;

		if (j == 0) {
			data.temp_1I = i;
			data.temp_1D = d;
		}
		else if (j == 1) {
			data.temp_2I = i;
			data.temp_2D = d;
		}

		//	Print to the serial port
		//	***************************************
		if (sensorConnected == 1) {			//	if there is a sensor connected then print the following
			char buffer[16];
			//	prepare the string to print for the version
			if (tempType == 0) { sprintf(buffer, "%s %3d.%dC", tempSensorNameGraph[j], i, d); }	//	print the type in C
			else sprintf(buffer, "%s %3d.%dF", tempSensorNameGraph[j], i, d);					//	print the type in F
			Serial.println(buffer);
		}
		else {
			Serial.print(addrString);		//	print the string for the address
			Serial.printf(", %s No Sensor Found", tempSensorNameGraph[j]);	//	print that no sensor is found
			Serial.println();
		}
	}
}

void FindTempSensors() {
	//	try 3 times to get the device count of the onewire pin
	for (byte j = 0; j < 3; j++) {
		numberOfSensors = tempSensors.getDeviceCount();		//	get the number of devices found on the bus

		//	if there were no sensors found, delay, print if needed, and repeat
		if (numberOfSensors == 0) {
			delay(500);
			Serial.printf("Did not find any sensors! Retry #%d", j);		//	print the retry message if sensor debuggint is on
		}
		//	this is for debugging, if it shows 5 sensors, it means that none were detected
		if (numberOfSensors == 0) { numberOfSensors = 4; }
	}
	//	Print the number of sensors on the pin to the serial port
	Serial.printf("Found %d temp sensors on pin %d\n", numberOfSensors, TEMP_SENSOR_PIN);

	// Search the wire for address
	for (byte j = 0; j < numberOfSensors; j++) {

		//	print the address to the serial port and set the resolution of the sensor
		if (tempSensors.getAddress(tempSensorAddr[j], j)) {
			//	print the found sensors address to the serial port
			String addr;	//	variable to store the address string
			Serial.printf("Found %s with address ", tempSensorNameGraph[j]);
			addr = convertTempSensorAddress(tempSensorAddr[j]);		//	convert the address to a string
			Serial.println(addr);		//	print the address string

			//tempSensors.setResolution(tempSensorAddr[j], TEMP_SENSOR_PRECISION);	// set the resolution to TEMPERATURE_PRECISION bit
		}
		else {
			//	if there isnt an address, print the following
			Serial.printf("Found ghost device at %d but could not detect address. Check power and cabling", j);
		}
	}
	Serial.println();		//	Print a line space for the next function
}

byte FindMyAddress()
// This function reads the 3 address pins and determines what address to use
{	
	byte add = 0;

	bitWrite(add, 0, digitalRead(ADDR_PIN_0));
	bitWrite(add, 1, digitalRead(ADDR_PIN_1));
	bitWrite(add, 2, digitalRead(ADDR_PIN_2));

	Serial.printf("Address = %d\n", add);

	return add;
}

String convertTempSensorAddress(DeviceAddress deviceAddress)
//	this function will convert the DS18B20 sensor address from a 64bit integer to a string that can be printed easily
//	devAddress = the address of the sensor that this function will print
{
	String addrString;		//	string to hold the address

	//	cycle through all 8 bytes of the sensor address
	for (uint8_t i = 0; i < 8; i++) {
		if (deviceAddress[i] < 16) addrString = addrString + "0";	//	zero pad the address if necessary
		addrString = addrString + String(deviceAddress[i], HEX);	//	add the current byte to the string
	}
	addrString.toUpperCase();	//	convert the string to all uppercase
	return addrString;			//	return the string
}