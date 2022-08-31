Swervo
======

Swervo is a low-cost swerve drive base for FRC off-season training.

<p float="left">
<img src="https://lh3.googleusercontent.com/pw/AM-JKLU5W324m551WcTt9s02GFsR1zLNU0ic61S2itBy-5KxmewNi9g5cKshvYVDg4Loodz1M26W2R2P9ljiseLwrPE0rsTpSK4iVKe3cX0hcxXtxNjtbingirLs51d0lNKdwlpfat-CVSvDiatPj2ZqSuCxpw=w1045-h784-no" width=400/>
<img src="https://lh3.googleusercontent.com/pw/AM-JKLXDs0E_ESYiH2j7yeR9WbBAzc0wz3PJryhOczWUMS4HV44MlBlDzr9S-CSX22fXicfVXHN69Xihr636AoKLMcYhotuPu6nFxobEB_ToFxmBrIdkn65ryjRN492NwnPuIk7K1QZI9s8WAABR3_eZmZpNhw=w1045-h784-no" width=400/>
</p>

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

Here's an example, executing a kinda-square path with a fixed aiming point:

<a href="https://www.youtube.com/watch?v=jptHhhOOdmU"><img src="https://img.youtube.com/vi/jptHhhOOdmU/0.jpg"></a>
          
