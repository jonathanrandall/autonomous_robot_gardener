### Parts list:
* Cytron MD10C R3 motor controllers X 4. Or you can 2 X dual cytron dual motor drivers. I would recommend cytron if you want to use my code an wiring.
* PCB or bread board. I printed up a PCB. The gerber files and fritzing files should be attached to this repo. I got this done at ![PCBWay](https://www.pcbway.com/).
* ESP32: I'm using the 38 pin waveshare esp32. This has a thinner form factor than the ones you get off Amazon. My PCB design should also fit the fatter 38 pin esp32. I got it here ![https://www.waveshare.com/nodemcu-32s.htm](https://www.waveshare.com/nodemcu-32s.htm)
4. Nylon stand offs. I would get two packs of these.
5. 6S lipo battery or 2 X 3S lipo batteries. I used two 3S lip batteries in series. Also will need one 3S for the top.
6. Battery mount. I 3D printed mine. But you can make one out of lego, since the grid plates on the gobilda system line up with the lego grids, it is easy to drill holes to attach.
7. Four pin JST XH connectors. These are for the motor encoders. I used these from GoBilda. But you can use any you like. ![SKU: 3802-0910-0300](https://www.gobilda.com/encoder-cable-extension-4-pos-jst-xh-300mm-length/)
8. Bullet extenders (female) to connect the motor to the motor encoders. I used these from GoBilda. ![SKU:3800-0013-0300](https://www.gobilda.com/3-5mm-bullet-lead-mh-fc-300mm-length/)
9. Missile Switch. I used the missile switch to control the power to the motors because it's easy to turn off in a hurry, and i liked the look of it. I attached it with lego.
10. Buck converter for 5V. I'm using this one [here](https://www.amazon.com.au/gp/product/B08RBXCJCF/)
11. High current buck converter for 12V. I'm using [this one](https://www.amazon.com.au/gp/product/B08KXTV6RH/), which outputs 20A max. 
12. Terminal strips X 4 (two for the top and two for the bottom). I used terminal strips with plastic cover and sixe connections. Six is the minimum. Make sure you get the ones with the plastic cover because if you drop a screw driver or something on it, you can cause a short.
13. Fork lugs for connecting the wires to the terminal strips.
14. Red and Black wires. Make sure you use red for positive and black for ground.
15. Dupont femal to male jumper wires X 14. Two to connect the pcb to grnd and positive. And 12 to connect the motor controllers to the PCB.
16. [__Hiwonder XARM esp32 X 2__](https://www.hiwonder.com/products/xarm-esp32)
17. __Raspberry pi 5.__
18. Raspberry pi 5 mount. I 3D printed [this one](https://www.printables.com/model/253933-simple-raspberry-pi-mount).
19. 3S battery.
20. __5A, 5V buck converter__ for raspberry pi (I used a non adjustable one). Make sure you get a good quality one, or else the pi won't start. I used __[tunghey 2Pack DC 9V 12V 24V to DC 5V 5A Buck Converter Module, 9-36V Step Down to USB 5V Transformer Dual Output Voltage Regulator Board](https://www.amazon.com.au/gp/product/B0D5V5YDC3/)__ from Amazon. The other alternative is to use a Waveshare UPS supply at 5V, but this is more expensive. I also tried an 8A buck converter but the pi wouldn't boot (probably because the voltage was a bit noisy?) and I tried a powerbank and the pi worked but with throttling on the CPU, which means it would run a bit slower. This is attached to the grid with a piece of lego, which is mounted on stand offs. The lego grid matches the goBilda grid plates, so it's easy to drill holes.
21. Adjustable Buck converter running at 8V to power the robot arms. I used this one: [DC5-30V to 1.25-30V Automatic Step DC-to-DC Power Converter UP/Down Converter Boost/Buck Voltage Regulator Module](https://www.amazon.com.au/gp/product/B08FXN9V68/), which is rated for high current and is fine with the robot arms, but didn't work with the pi.
22. __Stereo Camera__. I used [this one](https://www.amazon.com.au/gp/product/B07R8LQKV4/). Which is a Synchronized Dual Lens Stereo USB Camera 1.3MP HD 960P Webcam 3D VR Web Camera Module with 1/3 CMOS OV9715. Any stereo camera should work, but you need to make sure the left an right frames are synchronized, especially if the robot is moving.
23. Rocker switch.
24. Terminal strips X 4 (two for the top and two for the bottom). I used terminal strips with plastic cover and sixe connections. Six is the minimum. Make sure you get the ones with the plastic cover because if you drop a screw driver or something on it, you can cause a short.
25. Fork lugs to connect to the terminal strips.
26. Red and black wires (less than or equal 14 AWG).
27. Copper standoffs for XARM.
28. Nylong standoffs for 8V buck converter.
14. Male to male jumpber cables for connecting 8V buck converter to robot arm.



