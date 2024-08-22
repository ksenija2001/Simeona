# Motor control 

The code is divided into 3 modules, where each of them can be used by itself with minor adjustments:
- [Odometry](Robotic/Core/Src/odom.c)
- [UART](Robotic/Core/Src/uart.c)
- [Motor Control](Robotic/Core/Src/motor_control.c)

Each module takes advantage of different inbuilt functionalities of the STM32 Nucleo F411RE, which are also present in other Nucleo development boards and microcontrollers, in order to enable command driven movements - a Raspberry Pi mini-pc issues higher level commands.
These functionalities and how to set them up (so they actually work) are documented [here](Robotic/README.md).

For a more throughout explanation on why all of these modules are needed and how they fit in the whole project take a look at my [Bachelor project](docs/Diplomski_v3.pdf).

## Components and tools

The microcontroller STM32 Nucleo F411RE is used for motor control and communication with a RPi4. <br>
[STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) is used for programming and code deployment. 

Additional tools that provide easier debugging:
- [CuteCom](https://cutecom.sourceforge.net/) - serial terminal for debugging UART communication
- [STM32CubeMonitor](https://www.st.com/en/development-tools/stm32cubemonitor.html#get-software) - real time visualization of code variables, useful for generating time series graphs 
- [Logic](https://www.saleae.com/pages/downloads) - software used with a logic analyzer device that visualizes bits being transferred

STM32 microcontrollers allow the user to change peripheral functionalities based on their needs.
For example, a speicifc timer can be used in different modes that allow it to count up, or down, or count the impulses comming from an outside source connected to it's pin.

The Nucleo development board packaging provides easy access to all of the available peripherals, and is also compatible with different Arduino and STM32 shields that expand the possibilities of the microcontroller.
For our purposes the shield that enables the connection between the motors and their integrated encoders to the driver and to the microcontroller can be found [here](../shield).

The [reference manual](../docs/NUCLEOF411RE_reference_manual.pdf) for the sepcific MCU in use contains all information on alternate functions, registers, overall architecture and more, and will be cited frequnetly. 






