# STM32F407 DMA and ADC Test (Voltage and Current)
In this project, a voltage divider is used as a voltage sensor to measure the voltage value. Meanwhile, a current sensor (ACS712) is used as a current sensor to measure the current value. STM32F407 is used as the microcontroller with [ADC and DMA](https://www.digikey.com/en/maker/projects/getting-started-with-stm32-working-with-adc-and-dma/f5009db3a3ed4370acaf545a3370c30c) for handle moving (relatively) large amounts of data. The voltage and current values are shown on the LCD.
<br><br>
This is important, especially for **IoT energy monitoring projects in smart homes and smart industries**. An analog-to-digital converter (ADC) is a very useful feature that converts an analog voltage on a pin to a digital number. By converting from the analog world to the digital world, we can begin to use electronics to interface with the analog world around us. 
<br><br>
The ADC resolution can be defined as the smallest input voltage at the analog pin that an ADC can identify and increments its digital value. ADC resolution can be defined as:<br><br>
`Resolution = ( Operating voltage of ADC ) / 2^(number of bits ADC)`
<br><br>
![DMA Test](https://hackster.imgix.net/uploads/attachments/1416934/whatsapp_image_2021-12-23_at_22_29_14_%281%29_5nNgcbN01p.jpeg?auto=compress,format&w=740&h=555&fit=max)

## Softwares

 - [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
 - [Keil Embedded Development Tools µVision IDE](https://www.keil.com/download/)

## Hardware Components
|Components|Specification|Components|Specification|
|:--:|--|:--:|--|
|DC Power Supply|Vo 20V|LCD|20x4|
|ACS712 Current Sensor|5A|Voltmeter||
|Resistor|1kΩ|Amperemeter||
|Potentiometer|10kΩ|
