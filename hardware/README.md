Swervo Hardware
===============

The main goal for the hardware design is simplicity, and use of COTS parts, at the expense of, well, expense.
There are two fabricated parts:

* The base, made of [Owens Corning Foamular 150](https://www.owenscorning.com/en-us/insulation/products/foamular-150), an inexpensive lightweight rigid polystyrene foam which is [available locally](https://www.homedepot.com/p/Project-Panels-FOAMULAR-1-in-x-2-ft-x-2-ft-R-5-Small-Projects-Rigid-Pink-Foam-Board-Insulation-Sheathing-PP1/203553730) in small quantities in one-inch thickness.  It's resilient enough to serve as a bumper but strong enough to carry the rest of the parts.
* The module brackets, which are 3d-printed PLA.

Everything else is COTS, see the BOM.

There are Fusion 360 designs for both the [base](https://a360.co/3N4E435) and the [bracket](https://a360.co/3GCei3N), and also for the [module assembly](https://a360.co/3a7XJ3m) and the [full assembly](https://a360.co/3wX9a7f).

Hardware TODO's:

* 90mm wheels would produce 28% more speed than the 70mm, which might be nice.
* The Parallax 360 motors are expensive, constituting about half the system cost.  Consider alternatives?
  * The [FeeTech FB5317M-360](https://www.robotshop.com/en/feetech-digital-servo-15kg-cm-fb5116m-w-feedback.html) is under $20, but less than half the speed.
  * The [FeeTech FT5303R](https://www.robotshop.com/en/feetech-continuous-turn-digital-servo-3kg-cm-ft5303r.html) is under $10, but lacks an encoder.
  * The [DFRobot TT](https://www.dfrobot.com/product-1457.html) motor is $7.40 but lacks a driver.  DF drivers are only a few dollars per channel, but it's more complexity.  Also the TT gears are fragile.
  * The [TI-RSLK](https://www.pololu.com/product/3675), also used in the Romi, is $16 but lacks a driver.
* Steering could be faster, and the steering gears are expensive.  Consider alternatives for steering.  Printed gears?
* 6-channel slip rings are a few dollars cheaper than 12-channel, and would work, combining +/- wires for both motors.
