# Multimeter
The Target system is TIVA TM4C123GH6PM.<br>
The processor is 32-bit ARM M4F Processor (80 MHz). <br>
This is a simple LCR meter. The project uses Timers, Interrupts, A2D, Comparators and I2C protocols.<br>
The Hardware Layout is included as a LTSpice File.<br>
The Component to be measured is put in between the DUT1 and DUT2 and the respective calculations are carried out.<br>
There are two modes: UART mode and Pushbuttons Mode (Uses LCD with I2C I/O Expander). The calculations carried out are the same only the mode of output is different. <br>
The commands for UART mode are one character and are: <br>
1) r: Calculates resistance of the component. (Accurate for low resistances and high resistances, Error prone zone: 300 Ohms - 500 Ohms). <br> 
2) c: Calculates Capcitance of the component. (Accurate for low capacitances upto 0.1uF). <br>
3) e: Calculates the ESR of the component. (Also used in inductance measurement). <br>
4) l: Calculates the inductance of the component.
5) v: Calculates the voltage difference that a component creates. (Uses A2D) <br>
6) a: Automatically determines what a component is and calculates the respective value for the component.

