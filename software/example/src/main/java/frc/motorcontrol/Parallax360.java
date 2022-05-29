package frc.motorcontrol;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

/**
 * Continuous servo, see parallax.com/product/parallax-feedback-360-high-speed-servo/
 * 
 * This device expects 50Hz input, which is 4X period.
 * 
 * <ul>
 *   <li>1.720ms = "full forward"
 *   <li>1.520ms = high deadband
 *   <li>1.500ms = center deadband
 *   <li>1.480ms = low deadband
 *   <li>1.280ms = "full reverse"
 * </ul>
 * 
 * I used a scope to verify these bounds; they seem good.  Maybe 1.520 should be 1.525?
 */
public class Parallax360 extends PWMMotorController {
    public Parallax360(String name, int channel) {
        super(name, channel);
        m_pwm.setBounds(1.72, 1.52, 1.5, 1.48, 1.28);
        m_pwm.enableDeadbandElimination(true);  
        m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k4X);
        m_pwm.setSpeed(0.0);
        m_pwm.setZeroLatch();
    }

    public void setBounds(double fullAhead, double highDeadband, double center, double lowDeadband, double fullAstern) {
        m_pwm.setBounds(fullAhead, highDeadband, center, lowDeadband, fullAstern);
    }
}
