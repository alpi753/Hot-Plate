# Hot-Plate
A hotplate system which makes temperature sensing with RTD and heater power switching with SSR module. I moderated a old model CHILTERN Hotplate Magnetic Stirrer HS31.


![WhatsApp Image 2026-01-09 at 16 22 36](https://github.com/user-attachments/assets/25f87b35-6630-447f-885a-4614c9559560)

- 16x2 LCD Module  
- Rotery Enkoder  
- RTD driving circuit  
<https://electronics.stackexchange.com/questions/449575/working-of-ina826-instrumentation-amplifier-for-pt100-rtd>  
- STM32F103C8T6 Bluepill board 
***

### HOW TO USE:

#### Power:  
Use usb cable to power up the microcontroller with 5V,  
Plug in the heater and turn on the on/off swicth,  
You should adjust the heat setting at full extent (Speed setting is disabled in my case, i deattached its cable)  

#### Heating:  
To start heating operation, press short to the rotary enkoder.  
For adjusting temperature, press long to the rotary enkoder.  

### Programming:  
SWCLK ---  Green cable  
SWDIO ---  Yellow cable  
GND   ---  Blue cable  
3V3   ---  Orange cable  

(For native users, after me who will use the system which i used )
