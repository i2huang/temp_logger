/******************************************************************************

	 HTTP based temperature logger

    Copyright (C) 2020  Rick Huang

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*******************************************************************************/

#include <WiFi.h>
#include <EEPROM.h>
#include "wifi_config.h"

// Hardware configurations
#define DISCHARGE_RES_N		1				// Discharge sense resistor N, M  factor (N/M)
#define DISCHARGE_RES_M		2				// Resistor in fraction, 0.5 ohm = (1/2) = (N/M)
#define MAX_VBATT_SYSTEM   24000			// Maximum battery voltage supported by system

// ADC calibration 
#define ADC_0DB_VAR_A		77				// VAR_A = Offset, VAR_B = Slope
#define ADC_0DB_VAR_B		4153
#define ADC_6DB_VAR_A		110
#define ADC_6DB_VAR_B		2270
#define ADC_11DB_VAR_A		150
#define ADC_11DB_VAR_B		1220

// PIN config
#define PIN_TAN0				33	   		// Thermistor, 
#define PIN_TAN1				32    		// Thermistor,
#define PIN_TAN2				34	   		// Thermistor, 
#define PIN_TAN3				35    		// Thermistor,
#define PIN_V1					33    		// Voltage, 11:1 divider
#define PIN_CUR				34				// Current
#define TIMER_TICK_PIN		12				// For debugging
#define AP_MODE_PIN			15				// Go into AP mode if state is low

// Measurement settings
// Smallest interval is 18s as it is 5e-3h.  Anything smaller causes optimization error
#define MEAS_INTERVAL		18				// 18s per reading, cleaner dividing into 3600

// RunState definition
#define RUN_STATE_STOP		0
#define RUN_STATE_RUN		1				// Run discharge
#define RUN_STATE_END		2				// Continue to capture, no current
#define RUN_STATE_CLEAR		3				// Not a state, just a UI switch

// EEPROM layout
#define EEPROM_SIZE			40
#define WIFI_SSID_OFFSET   0
#define WIFI_SSID_SIZE     20
#define WIFI_PASS_OFFSET   20
#define WIFI_PASS_SIZE     20

// ********************************************
// Wifi network credentials
char Wifissid[20];
char Wifipassword[20];

// Web server port number 
WiFiServer server(80);

// ********************************************
// Global variables
int TimeTick = 0;								// Time tick in seconds
int TimeReadTick = 0;						// Ticks till next reading
int runState = 0;								// Start the test
int AvgDischargeMa;							// Discharge current average

// Current ADC readings
int vSense;										// Voltage in mV
int iSense;										// Current in mA
int temp0C;										// Temperature in degree C*10
int temp1C;										// Temperature in degree C*10
int temp2C;										// Temperature in degree C*10
int temp3C;										// Temperature in degree C*10

// Battery voltage reading over time
// Use uint16_t for data, 65V for voltage max, 65A for current max
//uint16_t vArray[10000];						// voltage reading over time
//uint16_t iArray[10000];						// current reading over time
int16_t t0Array[10000];						// Temp0 reading over time
int16_t t1Array[10000];						// Temp1 reading over time
int16_t t2Array[10000];						// Temp2 reading over time
int16_t t3Array[10000];						// Temp3 reading over time

int vDataArrayIdx;							// Current head of the data array
TaskHandle_t xDataTestHandle = NULL;	// Handle to data capture test task
int wifiApMode = 0;							// Using AP mode?

// ********************************************
// Function prototypes
void updateAdcReadings();

// ********************************************
// Timer ISR handler
volatile int timetickCounter;
volatile int g_wdt_cnt = 0;
hw_timer_t  *g_timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  timetickCounter++;
  // Watchdog timer, use by web service to timeout page
  g_wdt_cnt++;
  portEXIT_CRITICAL_ISR(&timerMux);
  // Signal to tasks
  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(xDataTestHandle != NULL) {
	  vTaskNotifyGiveFromISR(xDataTestHandle, &xHigherPriorityTaskWoken);
  }
}

// Convert mS number into S with decimal point
char *printMilli(uint32_t in)
{
	static char outbuf[20];
	char tempbuf[10]; 
	int mill, i;
	char *pt;
	mill = in%1000;
	itoa(in/1000, outbuf, 10);
	itoa(mill, tempbuf, 10);
	for (i=0;i<sizeof(outbuf);i++) {
		if (outbuf[i] == 0) break;
	}
	outbuf[i++] = '.';
	pt=tempbuf;
	if(mill >= 100) {
		outbuf[i++] = *pt++;
		outbuf[i++] = *pt++;
		outbuf[i++] = *pt++;
		outbuf[i++] = 0;
	} else if(mill >= 10) {
		outbuf[i++] = '0';
		outbuf[i++] = *pt++;
		outbuf[i++] = *pt++;
		outbuf[i++] = 0;
	} else {
		outbuf[i++] = '0';
		outbuf[i++] = '0';
		outbuf[i++] = *pt++;
		outbuf[i++] = 0;
	}
	return outbuf;
}

// Convert x10 number into x1 with decimal point
char *printTenth(uint32_t in)
{
	static char outbuf[20];
	char tempbuf[10]; 
	int frac, i;
	frac = in%10;
	itoa(in/10, outbuf, 10);
	itoa(frac, tempbuf, 10);
	for (i=0;i<sizeof(outbuf);i++) {
		if (outbuf[i] == 0) break;
	}
	outbuf[i++] = '.';
	outbuf[i++] = tempbuf[0];
	outbuf[i++] = 0;
	return outbuf;
}

// Plotly data output
void printPlotlyTable(WiFiClient &client)
{
	int i;
	const uint32_t timeitv = MEAS_INTERVAL * 1000/3600;
	String s;
	client.println("<!-- Load plotly.js into the DOM -->");
	client.println("<script src='https://cdn.plot.ly/plotly-latest.min.js'></script>");
	client.println("<script>");
 
	// Trace 1 - Temperature over time
	client.println("var trace1 = {");

	s = "x:[";
	for (i = 0; i < vDataArrayIdx; i++) {
		s += printMilli(i * timeitv);
		s += ",";
	}
	s += "],";
	client.println(s);

	s = "y:[";
	for (i = 0; i < vDataArrayIdx; i++) {
		s += printTenth(t0Array[i]);
		s += ",";
	}
	s += "],";
	client.println(s);

	client.println("type: 'scatter'};");

	// Trace 2 - Temperature over time
	client.println("var trace2 = {");

	s = "x:[";
	for (i = 0; i < vDataArrayIdx; i++) {
		s += printMilli(i * timeitv);
		s += ",";
	}
	s += "],";
	client.println(s);

	s = "y:[";
	for (i = 0; i < vDataArrayIdx; i++) {
		s += printTenth(t1Array[i]);
		s += ",";
	}
	s += "],";
	client.println(s);

	client.println("type: 'scatter'};");

	// Trace 3 - Temperature over time
	client.println("var trace3 = {");

	s = "x:[";
	for (i = 0; i < vDataArrayIdx; i++) {
		s += printMilli(i * timeitv);
		s += ",";
	}
	s += "],";
	client.println(s);

	s = "y:[";
	for (i = 0; i < vDataArrayIdx; i++) {
		s += printTenth(t2Array[i]);
		s += ",";
	}
	s += "],";
	client.println(s);

	client.println("type: 'scatter'};");

	// Trace 2 - Temperature over time
	client.println("var trace4 = {");

	s = "x:[";
	for (i = 0; i < vDataArrayIdx; i++) {
		s += printMilli(i * timeitv);
		s += ",";
	}
	s += "],";
	client.println(s);

	s = "y:[";
	for (i = 0; i < vDataArrayIdx; i++) {
		s += printTenth(t3Array[i]);
		s += ",";
	}
	s += "],";
	client.println(s);

	client.println("type: 'scatter'};");

	// Trace 1 function
	client.println("function plot_tempDiv() {");
	client.println("  var data = [trace1, trace2];");
	client.println("  var layout = {");
	client.println("    title: 'Temperature over time',");
	client.println("    xaxis: {");
	client.println("      title: 'Time (h)',");
	client.println("      zeroline: false");
	client.println("    },");
	client.println("    yaxis: {");
	client.println("    title: 'Temperature (C)',");
	client.println("  }};");
	client.println("  Plotly.newPlot('tempDiv', data, layout);");
	client.println("}");

	// Trace 2 function
	client.println("function plot_atempDiv() {");
	client.println("  var data = [trace3, trace4];");
	client.println("  var layout = {");
	client.println("    title: 'Temperature over time',");
	client.println("    xaxis: {");
	client.println("      title: 'Time (h)',");
	client.println("      zeroline: false");
	client.println("    },");
	client.println("    yaxis: {");
	client.println("    title: 'Temperature (C)',");
	client.println("  }};");
	client.println("  Plotly.newPlot('atempDiv', data, layout);");
	client.println("} </script>");
}

// CSV data formatted output
void printCsvData(WiFiClient &client)
{
	int i;
	client.println("Time, T0, T1, T2, T3");
	for (i = 0; i < vDataArrayIdx; i++) {
		String s = "";
		s += (i * MEAS_INTERVAL);
		s += ",";
		s += printTenth(t0Array[i]);
		s += ",";
		s += printTenth(t1Array[i]);
		s += ",";
		s += printTenth(t2Array[i]);
		s += ",";
		s += printTenth(t3Array[i]);
		client.println(s);
	}
}

void print404Page(WiFiClient &client)
{
	// HTTP headers start with a response code, in this case, 404
	client.println("HTTP/1.1 404 Not Found");
	client.println("Content-type:text/html");
	client.println("Connection: close");
	client.println();
	// End of HTTP header
	client.println("<html><head>");
	client.println("<title>404 Not Found</title>");
	client.println("</head><body>");
	client.println("Not Found");
	client.println("</body></html>");
}

// ********************************************
// Basic home page generation
void printHomePage(WiFiClient &client)
{
	int i;

	// HTTP headers start with a response code (e.g. HTTP/1.1 200 OK)
	client.println("HTTP/1.1 200 OK");
	client.println("Content-type:text/html");
	client.println("Connection: close");
	client.println();
	// End of HTTP header

	// HTML page header
	client.println("<!DOCTYPE html><html>");
	client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
	client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
	client.println("</style>");

	printPlotlyTable(client);
	client.println("</head>");
	
	// Core of the webpage
	client.println("<body><h1>Temperature of ... </h1>");
	client.println("<h2>Setup:</h2>");

	client.print("Run state: ");
	switch (runState) {
		case RUN_STATE_STOP:
			client.println("STOPPED<br>");
			break;
		case RUN_STATE_RUN:
			client.println("RUN<br>");
			break;
		case RUN_STATE_END:
			client.println("END<br>");
			break;
	}
	client.println("<br>");

	client.print("Current Temperature Reading (T0): ");
	client.print(printTenth(temp0C));
	client.println(" C <br>");
	client.print("Current Temperature Reading (T1): ");
	client.print(printTenth(temp1C));
	client.println(" C <br>");
	client.print("Current Temperature Reading (T2): ");
	client.print(printTenth(temp2C));
	client.println(" C <br>");
	client.print("Current Temperature Reading (T3): ");
	client.print(printTenth(temp3C));
	client.println(" C <br>");

	// Plot the output
	client.println("<div id='tempDiv'><script>plot_tempDiv()</script></div>");
	client.println("<div id='atempDiv'><script>plot_atempDiv()</script></div>");

	client.println("<a href='csv'>Data in CSV form</a>");
	
	client.println("</body></html>");
	
	// The HTTP response ends with another blank line
	client.println();
}

// ********************************************
// Data CSV page generation
void printCsvPage(WiFiClient &client)
{
	// HTTP headers start with a response code (e.g. HTTP/1.1 200 OK)
	client.println("HTTP/1.1 200 OK");
	client.println("Content-type:text/html");
	client.println("Connection: close");
	client.println();
	// End of HTTP header

	// HTML page header
	client.println("<!DOCTYPE html><html>");

	// Core of the webpage
	client.println("<body>");

	// Loop through the output array and display 
	client.println("<pre>");
	printCsvData(client);
	client.println("</pre>");
	
	// The HTTP response ends with another blank line
	client.println();
}

// ********************************************
// Settings page generation
void printSettingsPage(WiFiClient &client)
{
	// HTTP headers start with a response code (e.g. HTTP/1.1 200 OK)
	client.println("HTTP/1.1 200 OK");
	client.println("Content-type:text/html");
	client.println("Connection: close");
	client.println();
	// End of HTTP header

	// HTML page header
	client.println("<!DOCTYPE html><html>");
	client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
	client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: left;}");
	client.println("</style>");

	client.println("</head>");
	
	// Core of the webpage
	client.println("<body><h1>Data Capture configurations</h1>");

	client.println("<form action='/config' method='get'>");

	client.print("WiFi SSID: <input type='text' name='ssidSet' value='");
	client.print(Wifissid);
	client.println("'> <br>");
	client.print("WiFi Password:  <input type='text' name='wifipassSet'");
	client.println("'> <br>");

	client.println("<input type='submit' value='Update WiFi Settings'>");
	client.println("</form>");
	client.println("<br>");
	client.println("<br>");

	client.println("<form action='/config' method='get'>");

	client.println("Run State: <br>");
	if (runState == RUN_STATE_STOP || runState == RUN_STATE_END) {
		client.println("<input type='radio' name='runStartSet' value='0' checked> Stopped<br>");
	} else {
		client.println("<input type='radio' name='runStartSet' value='0'> Stopped<br>");
	}
	if (runState == RUN_STATE_RUN) {
		client.println("<input type='radio' name='runStartSet' value='1' checked> Run capture<br>");
	} else {
		client.println("<input type='radio' name='runStartSet' value='1'> Run capture<br>");
	}
	client.println("<input type='radio' name='runStartSet' value='3' > Reset<br>");
	client.println("<input type='submit' value='Set run state'>");
	client.println("</form>");
	client.println("</body></html>");
	
	// The HTTP response ends with another blank line
	client.println();
}

// Convert input in mV to output in 10xT
int convVoltageToTemp(int mV)
{
	float Rt, Temp;

	Rt = 50000.0 / (3300.0 / mV - 1.0);
	Temp = 1/3950.0 * log (Rt / 50000.0);
	Temp = 1.0/( Temp + 1.0 / (25+273)) - 273;

	return (Temp * 10);
}

// Read all the ADC values and store in global
void updateAdcReadings() 
{
	int sensorValue;
#if 0
	analogSetPinAttenuation(PIN_V1, ADC_0db);
	analogSetPinAttenuation(PIN_CUR, ADC_6db);
	sensorValue = analogRead(PIN_V1);
	sensorValue += analogRead(PIN_V1);
	sensorValue += analogRead(PIN_V1);
	sensorValue += analogRead(PIN_V1);
	sensorValue /= 4;
	sensorValue = sensorValue * 1000 / ADC_0DB_VAR_B + ADC_0DB_VAR_A;
	iSense = sensorValue;

	sensorValue = analogRead(PIN_CUR);
	sensorValue += analogRead(PIN_CUR);
	sensorValue += analogRead(PIN_CUR);
	sensorValue += analogRead(PIN_CUR);
	sensorValue /= 4;
	vSense = sensorValue * 1000 / ADC_6DB_VAR_B + ADC_6DB_VAR_A;
	vSense = vSense * 11;							// divider ratio
#endif

	sensorValue = analogRead(PIN_TAN0);						
	sensorValue = sensorValue * 1000 / ADC_11DB_VAR_B + ADC_11DB_VAR_A;
	temp0C = convVoltageToTemp(sensorValue);

	sensorValue = analogRead(PIN_TAN1);
	sensorValue = sensorValue * 1000 / ADC_11DB_VAR_B + ADC_11DB_VAR_A;
	temp1C = convVoltageToTemp(sensorValue);

	sensorValue = analogRead(PIN_TAN2);
	sensorValue = sensorValue * 1000 / ADC_11DB_VAR_B + ADC_11DB_VAR_A;
	temp2C = convVoltageToTemp(sensorValue);

	sensorValue = analogRead(PIN_TAN3);
	sensorValue = sensorValue * 1000 / ADC_11DB_VAR_B + ADC_11DB_VAR_A;
	temp3C = convVoltageToTemp(sensorValue);
}

// Data capture routine
void runSingleCheck ()
{
	//vArray[vArrayIdx] = vSense;
	//iArray[vArrayIdx] = iSense;
	t0Array[vDataArrayIdx] = temp0C;
	t1Array[vDataArrayIdx] = temp1C;
	t2Array[vDataArrayIdx] = temp2C;
	t3Array[vDataArrayIdx] = temp3C;
	vDataArrayIdx++;
}


// HTTP URL command processor
void processUrlCommands(String &header)
{
	int idxRunStart = header.indexOf("runStartSet=");
	int idxWifissid = header.indexOf("ssidSet=");
	int idxWifipass = header.indexOf("wifipassSet=");

	// Process the form data and update internal variables
	// idxWifissid
	if (idxWifissid >= 0) {
		String s = header.substring(idxWifissid + 8, idxWifissid + 8 + 20);
		int i = s.indexOf("&");
		s = s.substring(0, i);
		Serial.print("Wifi SSID set ");
		Serial.println(s);

		for (i=0;i<WIFI_SSID_SIZE;i++) {
			 EEPROM.write(i + WIFI_SSID_OFFSET, s.charAt(i));
			 if (s.charAt(i) == 0) break;
		}
	}

	// idxWifipass
	if (idxWifipass >= 0) {
		String s = header.substring(idxWifipass + 12, idxWifipass + 12 + 20);
		int i = s.indexOf(" ");
		s = s.substring(0, i);
		Serial.print("Wifi PASSWD set ");
		Serial.println(s);

		// Remember the Wifi information
		for (i=0;i<WIFI_PASS_SIZE;i++) {
			 EEPROM.write(i + WIFI_PASS_OFFSET, s.charAt(i));
			 if (s.charAt(i) == 0) break;
		}
#ifdef USE_CONFIG_UI_WIFI
		EEPROM.commit();
#endif
	}

	// isRunStart
	if (idxRunStart >= 0) {
		int newRunStart;
		String s = header.substring(idxRunStart + 12, idxRunStart + 12 + 2);
		newRunStart =  s.charAt(0) == '1' ? 1 : 
							s.charAt(0) == '4' ? 4 : 0;
		if (s.charAt(0) == '3') {
			// runStart of 3 is a reset of captured data
			Serial.println("Clear captured data");
			portENTER_CRITICAL_ISR(&timerMux);
			timetickCounter = 0;
			portEXIT_CRITICAL_ISR(&timerMux);
			AvgDischargeMa = -1;
			vDataArrayIdx = 0;
			runState = RUN_STATE_STOP;
		}
		if (runState == RUN_STATE_STOP && 
			 (newRunStart == RUN_STATE_RUN)) {
			// Start a new test run
			Serial.println("Starting a new test run");
			TimeReadTick = timetickCounter + MEAS_INTERVAL;
		}
		runState = newRunStart;
		Serial.print("runStart ");
		Serial.println(runState);
	}
}

// Web server thread
void webServerProcess(void *id) 
{
	int reconnect_retry = 10;
	while (1) {
		int isCaptureHeader = 1;
		String header = "";								// HTTP header
		WiFiClient client = server.available();   // Listen for incoming clients
		g_wdt_cnt = 0;

		if(!wifiApMode) {
			while (WiFi.status() != WL_CONNECTED) {	
				// Disconnect happened, try to reconnect
				if (reconnect_retry == 0) {
					WiFi.begin(Wifissid, Wifipassword);
					reconnect_retry = 10;
				}
				delay(500);
				reconnect_retry--;
			}
		}

		if (client) {
			Serial.println("New Client.");			// Client connected
			int currentLine = 0;
			while (client.connected()) {           // loop while the client's connected
				if (g_wdt_cnt >= 5) {					// 5s connection timeout
					Serial.println("Connection timeout");	
					break;
				}
				if (client.available()) {           // Read any available byte and store it
					g_wdt_cnt = 0;
					char c = client.read();
					//Serial.write(c);               // Debug loopback
					if (isCaptureHeader) 
						header += c;

					if (c == '\n') {                 // if the byte is a newline character
						// if the current line is blank, you got two newline characters in a row.
						// that's the end of the client HTTP request, so send a response:
						if (currentLine == 0) {
							// Capture the time at start, profile page generation time
							TickType_t start_t, end_t;
							start_t = xTaskGetTickCount();

							Serial.println(header);			// Print out the HTTP heade

							// Locate all the keywords
							int idxStatusHtml = header.indexOf("status"); 
							int idxConfigHtml = header.indexOf("config"); 
							int idxCsvHtml = header.indexOf("csv"); 
							
							if (idxConfigHtml >= 0) {
								processUrlCommands(header);
								printSettingsPage(client);
							}
							else if (idxStatusHtml >= 0) {
								// Print the HTML page and exit the loop
								printHomePage(client);
							}
							else if (idxCsvHtml >= 0) {
								printCsvPage(client);
							}
							else {
								// Print 404 error
								print404Page(client);
							}
							end_t = xTaskGetTickCount();
							Serial.print("Page load time: ");
							Serial.print((end_t - start_t) * portTICK_PERIOD_MS);
							Serial.println("mS");
							break;
						} else { // if you got a newline, then clear currentLine
							isCaptureHeader = 0;
							currentLine = 0;
						}
					} else if (c != '\r') {  
						currentLine++;			// If not '\r', increase currentLine
					}
				}
			}
			// Close the connection
			client.stop();
			Serial.println("Client disconnected.");
			Serial.println("");
		}
		taskYIELD();
	}
}

// Data test thread
void dataTestProcess(void *id) 
{
	// ********************************************
	// Variable to store the HTTP request
	int LastTimeTick;
	//const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;		// 1s delay

	while (1) {
		// Run battery test loop, move to its own thread later
		// Read the current ADC values
		uint32_t timeout = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
		digitalWrite(TIMER_TICK_PIN, HIGH);
		if(LastTimeTick != timetickCounter) {
			updateAdcReadings();
		}
		if (TimeReadTick <= timetickCounter) {
			if (runState != RUN_STATE_STOP) {
				runSingleCheck();
				TimeReadTick += MEAS_INTERVAL;
			}
		}
		digitalWrite(TIMER_TICK_PIN, LOW);
		if (!timeout) {
			Serial.println("Data test timer missing signal");
		}
	}
}

// ********************************************
void setup() 
{
	Serial.begin(115200);

	pinMode(TIMER_TICK_PIN, OUTPUT);
	digitalWrite(TIMER_TICK_PIN, LOW);
	pinMode(AP_MODE_PIN, INPUT);

	EEPROM.begin(EEPROM_SIZE);

	// Store default ssid/password
	// Use this till Wifi Configuration interface is added
#ifdef USE_SET_DEFAULT_WIFI
	{
		int i;
		const char *defssid = DEFAULT_WIFI_SSID;
		const char *defpasswd = DEFAULT_WIFI_PASSWD;
		for (i=0;i<WIFI_SSID_SIZE;i++) {
			 EEPROM.write(i + WIFI_SSID_OFFSET, defssid[i]);
		}
		for (i=0;i<WIFI_PASS_SIZE;i++) {
			 EEPROM.write(i + WIFI_PASS_OFFSET, defpasswd[i]);
		}
		EEPROM.commit();
	}
#endif

	// Recall SSID/PASSWD
	int i;
	for (i=0;i<WIFI_SSID_SIZE;i++) {
		 Wifissid[i] = EEPROM.read(i + WIFI_SSID_OFFSET);
	}
	for (i=0;i<WIFI_PASS_SIZE;i++) {
		 Wifipassword[i] = EEPROM.read(i + WIFI_PASS_OFFSET);
	}
	
	wifiApMode = !(digitalRead(AP_MODE_PIN));
	if (!wifiApMode) {
		// Connect to Wi-Fi network with SSID and password
		Serial.print("Connecting to ");
		Serial.println(Wifissid);

		WiFi.begin(Wifissid, Wifipassword);
		while (WiFi.status() != WL_CONNECTED) {
		  delay(500);
		  Serial.print(".");
		}

		Serial.println("");
		Serial.println("WiFi connected.");
		Serial.println("IP address: ");
		Serial.println(WiFi.localIP());
	} else {
		// Soft AP mode
		WiFi.mode(WIFI_AP);
		Serial.print("Setting up AP");
		Serial.println(DEFAULT_WIFIAP_SSID);
		WiFi.softAP(DEFAULT_WIFIAP_SSID, DEFAULT_WIFIAP_PASSWD);
		Serial.println("IP address: ");
		Serial.println(WiFi.softAPIP());
	}

	// Starting the HTTP server
	server.begin();

	// Setup 1s counter
	g_wdt_cnt = 0;
	timetickCounter = 0;
	g_timer = timerBegin(0, 80, true);
	timerAttachInterrupt(g_timer, &onTimer, true);
	timerAlarmWrite(g_timer, 1000000, true);			// Trigger the timer once a second
	timerAlarmEnable(g_timer);

	// Test variables setup
	vDataArrayIdx = 0;

	TaskHandle_t xHandle = NULL;
	// Create the threads
	xTaskCreate( webServerProcess,  /* Function */
                "webServer",       /* Task name. */
                4000,              /* Stack size in words, not bytes. */
                NULL,              /* Parameter */
                tskIDLE_PRIORITY,  /* Priority */
                &xHandle );        /* Task's handle. */

	xTaskCreate( dataTestProcess,   /* Function */
                "dataTest",        /* Task name. */
                2000,              /* Stack size in words, not bytes. */
                NULL,              /* Parameter */
                tskIDLE_PRIORITY + 2,  /* Priority */
                &xDataTestHandle );/* Task's handle. */
}

// Empty loop, nothing is done here, idle the task
void loop()
{
	vTaskDelay(portMAX_DELAY);
}


