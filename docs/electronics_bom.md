### Parts list:

#### __Major Components__
* [Hiwonder XARM esp32](https://www.hiwonder.com/products/xarm-esp32)
* Mini PC. I'm using [this one from Amazon](https://www.amazon.com.au/dp/B0CGZX9M96), but shop around.
* RPLidar module
* Time of flight Camera (I'm using one from [Arducam](https://blog.arducam.com/time-of-flight-camera-raspberry-pi/))
* USB rgb camera (I'm using one from Arducam)
* Raspberry pi 4 (to connect to the time of flight camera)
* Servo pan-tilt mechanism to mount the cameras on

__Note:__ you can use something like an intellisense camera instead of the time of flight and rgb camera combination. 

#### __Control & Electronics__
* Cytron MD10C R3 motor controllers X 4. Or you can 2 X dual cytron dual motor drivers. I would recommend cytron if you want to use my code an wiring.
* PCB or bread board. I printed up a PCB. The gerber files and fritzing files should be attached to this repo. I got this done at ![PCBWay](https://www.pcbway.com/).
* ESP32: I'm using the 38 pin waveshare esp32. This has a thinner form factor than the ones you get off Amazon. My PCB design should also fit the fatter 38 pin esp32. I got it here ![https://www.waveshare.com/nodemcu-32s.htm](https://www.waveshare.com/nodemcu-32s.htm)
* Nylon stand offs. I would get two packs of these.
* Four pin JST XH connectors. These are for the motor encoders, or use whatever is specified for your motor. 
* Bullet extenders (female) to connect the motor to the motor encoders. I used these from GoBilda. ![SKU:3800-0013-0300](https://www.gobilda.com/3-5mm-bullet-lead-mh-fc-300mm-length/)
* Dupont jumper wires. 

#### __Powering the robot__
* 3s lipo battery for the electronics.
* 6S lipo battery for the motors.
* Battery for mini-pc
* Missile Switch for the 6s lipo. I used the missile switch to control the power to the motors because it's easy to turn off in a hurry, and i liked the look of it. I attached it with lego.
* Rocker Switch for the 3s lipo.
* High current buck converter for 12V motor power. I'm using [this one](https://www.amazon.com.au/dp/B0BRWD2VRT), which outputs 20A max. 
* Buck converter for servos for robot arm and camera pan tilt. These run at 7.4V and 6.8V respectively. I'm using this one [here](https://www.amazon.com.au/gp/product/B08RBXCJCF/)
* 5A, 5V buck converter for raspberry pi (I used a non adjustable one). Make sure you get a good quality one, or else the pi won't start. I used [tunghey 2Pack DC 9V 12V 24V to DC 5V 5A Buck Converter Module, 9-36V Step Down to USB 5V Transformer Dual Output Voltage Regulator Board](https://www.amazon.com.au/gp/product/B0D5V5YDC3/) from Amazon. The other alternative is to use a Waveshare UPS supply at 5V, but this is more expensive. 
* Terminal strips or connectors for. See [here](https://www.amazon.com.au/dp/B0B75J8768) or [here](https://www.amazon.com.au/dp/B0DFM5FNYN) for examples.
* Fork lugs for connecting the wires if using terminal strips.
* Red and Black wires. Make sure you use red for positive and black for ground.
* Copper and/or nylon standoffs. I used 3M.












