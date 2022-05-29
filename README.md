Swervo
======

Swervo is a low-cost swerve drive base for FRC off-season training.

![early version driving](https://photos.app.goo.gl/ufzaHw8fnDydqWgt5)

It enables hardware and software teams to iterate and experiment with
issues like trajectory following, control tuning, and it can serve as
a base for other projects like targeting, at a much lower price point
than "real" swerve: if you have an extra RIO, a few hundred dollars in parts is
all you need.  Unlike other low-cost platforms, Swervo uses a real RIO,
so teams should be able to use the very same code on the large and small
platforms, with just configuration/tuning differences.

The design goal is simplicity, not lowest-possible cost.  Accordingly, the two key enablers are:

* The [Parallax 360 continuous servo](https://www.parallax.com/product/parallax-feedback-360-high-speed-servo/), which includes an encoder,
and which can be driven directly by the RIO at about 120 RPM. 
* [Slip rings](https://www.sparkfun.com/products/13065), which allow 
direct drive.

Both are expensive parts, but they make the design very simple: no motor controllers, no PDP, no encoders, no linkages, etc.
