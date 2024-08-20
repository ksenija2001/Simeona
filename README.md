# Motor control 

The code is divided into 3 modules:
- [Odometry](Roobotic/Core/Inc/odom.h)
- [UART](Robotic/Core/Inc/uart.h)
- [Motor Control](Robotic/Core/Inc/motor_control.h)

Each module takes advantage of different inbuilt functionalities which can be found on most of their microcontrollers.
These functionalities and how to set them up so they actually work are documented [here](Robotic/README.md).

For a more throughout explanation on why all of these modules are needed and how they fit in the whole project take a look at my [Bachelor project](docs/Diplomski_v3.pdf).

## Components and tools

The microcontroller STM32 Nucleo F411RE is used for motor control and communication with a RPi4. <br>
[STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) is used for programming and code deployment. 

Additional tools that provide easier debugging:
- [CuteCom](https://cutecom.sourceforge.net/) - serial terminal for debugging UART communication
- [STM32CubeMonitor](https://www.st.com/en/development-tools/stm32cubemonitor.html#get-software) - real time visualization of code variables, useful for generating time series graphs 
- [Logic](https://www.saleae.com/pages/downloads) - software used with a logic analyzer device that visualizes bits being transferred







