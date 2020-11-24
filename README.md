# MMU2-DIP
Ported from [pruas3D](https://github.com/prusa3d/MM-control-01)  
Mcu: stm32f0c8  
Supports all types of drivers for bigtreetech,  
Supports TMC stall protection (tmc2209,tmc2130),  
Supports manual loading of filaments,  

# Modify configuration  
In MMU2/src/config.h  
Set the drive type, motor current, etc.

# Building  
It is recommended to use vscode to install the PlatformIO plugin for compilation.  


# Upgrade firmware in Windows  
1. Install Tool/cd340driver/CH341SER.EXE.  
2. Connect the usb  
3. Open Tool/flymcu/FlyMcu.exe.  
  ①. refreshing the port.  
  ②. select the corresponding port.  
  ③. setting the baud rate.  
  ④. Select the firmware.hex file.  
  ⑤. Set download parameters.  
  ⑥. click to start downloading.  
  ![image](https://github.com/MaiEmily/map/blob/master/public/image/20190528145810708.png)