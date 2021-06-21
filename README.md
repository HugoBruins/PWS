## Information

This repository was made for our Dutch school (finals) project, I changed the readme to English so everyone would understand, but all the comments in the code are still made in Dutch, because otherwise it wouldn't match the project anymore.

For our finals project, we made a self-balancing robot where we explored the magic of PID control. Our project actually won first place for a Dutch competition for best computer science final school project (In Dutch this would be called profielwerkstuk). [3i award first place vwo](https://3i-award.nl/prijswinnaars-3i-award-2021/ "3i award first place vwo").

## Arduino Code

For the Arduino code, you will need to install the libraries 'Accelstepper' and Wire.h and upload to an Arduino Nano via the IDE software.

'Uiteindelijke_code.ino' is what our final Arduino code was.

## Python Code

In chapter 1 of our school project we used figures to explain PID, these figures have the same numbers as the python programs used to make them. 

#### Dependencies

- MatPlotLib
- Numpy

#### Instructions to recreate the plots

1. Check what figure you want to recreate, search the code with the same name (say figure 12, load `PID fig12.py`).
2. Run the code.
3. Change the variable values (mostly the P, I and D values) to match the second line of the plot (for figure 12 specifically you will need to uncomment lines `24` & `25` of the code).
4. Run the code.
5. Change the constants and run the code again (without closing the plot) if there are multiple lines in one graph. 

