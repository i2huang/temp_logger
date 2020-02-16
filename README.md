# temp_logger
A ESP32 based temperature logger
<br>
A simple temperature data logger that is connected on the WiFi that serve a webpage. <Br>
Please find out the address from the serial output and replace the 192.168.x.x with the correct IP address of the ESP32.  If the ESP32 is placed in AP mode, the address is fixed to 192.168.4.1. <br>
Basic address is: <br>
http://192.168.x.x/status This is the status page  <br>
http://192.168.x.x/config This is the config page  <br>
<br>

## Example output
Here is an example of how the output looks:
![GitHub Logo](/status_view.png)

## Circuit
Circuit is pretty simple.  Thermistor input requires 50Kohm pull up resistor to 3.3V.  I added a 0.1uF capacitor on each of the input GPIO to reduce the noise on the signal sampled. <br>
If you would like to use the AP mode feature, add a switch on GPIO 15.<br>
The GPIO used for the project can be any of the available ADC capable GPIO lines.  However, only the ADC1 lines can be used as ADC2 are used by the WiFi function.<br>
![GitHub Logo](/schematic.jpg)

## Thermo probe
The thermistor I used for the project is "MF58 3950B 50Kohm," available from Amazon. <br>
Make a simple probe by plugging in the thermistor into a hookup wire.
![GitHub Logo](/probe.jpg)
