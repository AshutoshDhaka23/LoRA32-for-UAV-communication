# LoRA32-for-UAV-communication
Using LoRA an LPWAN technology for UAV communication 

setting up the workspace
### adding the node mcu board 
paste this in the arduino ide - http://arduino.esp8266.com/stable/package_esp8266com_index.json 
boards manager look for esp8266 (3.1.2) the version I am using 

### Lora library
https://github.com/jpmeijers/RN2483-Arduino-Library - using this lib example we can setup a basic connection to the gateway in DTU. 

### setting up the simulation 
https://ardupilot.org/dev/docs/building-setup-linux.html - this is what is needed. 
and using https://mavlink.io/en/mavgen_python/ - this mavlink reference a basic message parser can be made.

https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html - this link is really important to simulate what we want. 