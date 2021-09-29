# RcBoatController

## Overview

## Hardware

### History
The boat frame has been apparently bought originally from Biltema some years ago. At some point, the radio controller had been lost, as well as the battery. The boat body was then handed by the original owner to my father-in-law who is known as a man who can fix anything. The boat was stored for some time as a "future project" and then handed to me later.
I googled a bit and didn't find much info of the boat, neither were the suitable radio controllers available on ebay. So, it started look like a DIY project.

### Boat frame
The boat frame is made of plastic and being relatively big, it has plenty of space for new things. The propellers, axles and motors were in good shape, so I saved them and removed the rest: original speed controller and wiring.

<img src="doc/photos/frame1.jpg" alt="boat frame 1" width="30%"/>
<img src="doc/photos/frame2.jpg" alt="boat frame 2" width="30%"/>
<img src="doc/photos/rearside.jpg" alt="boat frame 3" width="30%"/>

### Motors
The model/manufacturer of motors is not known, but they are pretty normal brushed DC motors. Using a lab power supply, it looks like they operate nicely at around 8 volts. More than that makes them run "too fast". Unfortunately, my power supply can deliver only 5A, so I was not able to measure the peak current consumption of the power line, but the motor behaviour seems to be significantly bigger than 5A per motor. Naturally, the current consumption is dependent on the situation. At start-up and in low duty cycle PWM drive, the current consumption is jumpy.


### Power supply
Knowing the desired voltage being ~8V and current consumption being more than 5A per motor, choosing li-ion cells in 2S configuration was quite a clear choice. For safety purpose, I prefer using protected cells. On the other hand, those protection circuits typically limit the output current. The cells I chose were rated at 10A max, so I decided to use four of them in 2S2P configuration. This setup should be fine to keep the motors running nicely.

### Radio control
I was told that the original product had a two lever controller, supposed to be used by two hands - one controlling the left side motor, the other one controlling the right side motor. I'm not a big fan of such control, so it was natural to go for a bit more user friendly approach for the replacement.
I found a controller+receiver kit for RC cars (see [here](https://www.rhs.fi/rc-tarvikkeet-1/radiolaitteet/radio/muut/maxam-e0105-xt-16-2-4ghz-3-kanals-rattradio.html)).

<img src="doc/photos/rhs_controller.jpg" alt="radio controller from rhs.fi" width="20%"/>
<img src="doc/photos/rhs_receiver.jpg" alt="radio receiver from rhs.fi" width="20%"/>

The kit works well and it was easy to interact with. The datasheet didn't tell much, but the output signals connected to an oscilloscope made it clear. Both channels provide a periodic pulse at 60Hz interval. The length of the pulses vary between ~1000us...~2000us, which made it straight forward to convert the pulse lengths to lever percentages.

### Power output circuit
At first, I used [DRV8871](https://www.ti.com/product/DRV8871)-based H-bridges to run the motors. They almost worked, but the driver chip got overheated and the current limiting protection activated too often, causing the motor to get jumpy and slow down occasionally during run.
After measuring the current consumption, it became clear that stronger driver circuits are needed. I ended up using [BTS7960](https://www.partco.fi/en/electromechanics/motors/motor-controllersdrivers/19263-bts7960b-modul.html)-based bridges, one per motor. They are too big for this purpose, but the boat benefits from some mass and they fit nicely in the middle part of the boat, so let them be. I have never got the cooling elements to heat up noticeably, so I almost believe they could be run even without the heat sinks in this use case.

<img src="doc/photos/poweroutput.jpg" alt="power output bridges" width="40%"/>


### Speed feedback sensor
Being an engineer who enjoys over solved problems, I decided to add speed encoders to both axles to enable the controlled speed balancing of the motors. The main goal was to get the boat to run straight when not intentionally steered anywhere.

There was no space or suitable holder for mounting a normal rotary encoder disc, so I decided to create a DIY version of it. There were some [NERF darts](https://www.hasbro.com/common/productimages/fi_FI/CCCE147C5056900B10553FA593C070AD/converted1ffcb02be85a88a948de298296f6b2eaec526f63.jpg) available at my house, so I used one of them. Painted in matte black, glued with four narrow stripes of aluminium foil and sliding it on the axle, I ended up having something that can be used as an encoder. The only thing needed was an optical detector, where a pair of [CNY70's](https://www.vishay.com/docs/83751/cny70.pdf) became handy. One revolution makes the detector to see eight state transitions (dark->lit->dark->lit etc), which can then be counted by the microcontroller and converted to speed. Naturally, this kind of feedback sensor is not aware of direction of movement, but for the given use case, the directionless speed information was considered enough.

<img src="doc/photos/speeddetector.jpg" alt="speed detector CNY70 mounted on a holder plate" width="40%"/>

<a href="https://youtu.be/PrkZCHm4Xqk"><img src="doc/photos/preview_speeddetector.jpg" alt="speed detector" width="40%"/></a>


### Test Drive

<a href="https://youtu.be/F3i1ZTXYGnA"><img src="doc/photos/preview_testdrive.jpg" alt="test drive" width="40%"/></a>






## Software

### Build environment

### External libraries

### System view

### Modules

#### MotorDriver

#### Controller

#### RcReceiver

#### SpeedEncoder

#### BatteryMonitor

#### Common


