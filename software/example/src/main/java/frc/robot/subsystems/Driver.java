package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.motorcontrol.Parallax360;

public class Driver extends PIDSubsystem {

    private static final double kWheelDiameterMeters = 0.07;
    private static final double kV = 0.4; // motor output per turns/sec
    private static final int kP = 0;
    private static final double kI = 4;
    private static final double kD = 0;
    private static final double kMaxVelocityTurnsPerSec = 2.1;

    public final Parallax360 m_motor;
    public final DutyCycleEncoder m_input;
    private final LinearFilter m_dx;
    private final LinearFilter m_dt;
    private double m_currentDx; // position units = meters
    private double m_currentDt; // time units = microsec
    private double m_feedForwardOutput;
    private double m_controllerOutput;
    private double m_velocityMetersPerSec;
    private double m_userInput; // [-1,1]

    /**
     * @param channel for PWM motor and duty cycle encoder.
     */
    public Driver(int channel) {
        super(new PIDController(kP, kI, kD), 0);
        setName(String.format("Drive %d", channel));
        m_motor = new Parallax360(String.format("Drive Motor %d", channel), channel);
        m_input = new DutyCycleEncoder(channel);
        m_input.setDutyCycleRange(0.027, 0.971);
        m_input.setDistancePerRotation(-1 * Math.PI * kWheelDiameterMeters); // wheels are 70mm diameter
        // getController().enableContinuousInput(0, 1);
        getController().setTolerance(0.01, 0.01);
        // these gains yield dx per period, averaged over a few periods.
        double[] gains = new double[5];
        gains[0] = 0.25;
        gains[4] = -0.25;
        m_dx = new LinearFilter(gains, new double[0]);
        m_dt = new LinearFilter(gains, new double[0]);
        enable();
        SmartDashboard.putData(getName(), this);
    }

    /**
     * Setpoint in meters per second.
     */
    @Override
    public void setSetpoint(double setpoint) {
        super.setSetpoint(setpoint);
    }

    /**
     * Distance in meters. this is actually distance, odometry.
     */
    public double getDistance() {
        return m_input.getDistance();
    }

    /**
     * Motor units [-1, 1]
     */
    public void setMotorOutput(double value) {
        m_motor.set(value);
    }

    /**
     * @param output   Output is what the controller thinks, in motor units [-1, 1]
     * @param setpoint Setpoint is the velocity we want to end up with, meters per
     *                 sec
     */
    @Override
    protected void useOutput(double output, double setpoint) {
        m_controllerOutput = output;
        m_feedForwardOutput = kV * setpoint / (Math.PI * kWheelDiameterMeters);
        // m_feedForwardOutput = 0;
        setMotorOutput(m_controllerOutput + m_feedForwardOutput);
    }

    /**
     * Velocity in meters per second, average over the past 80ms.
     */
    @Override
    protected double getMeasurement() {
        m_currentDt = m_dt.calculate(RobotController.getFPGATime());
        m_currentDx = m_dx.calculate(getDistance());
        m_velocityMetersPerSec = 1e6 * m_currentDx / m_currentDt;
        return m_velocityMetersPerSec;
    }

    /**
     * an old control method, input [-1,1]
     */
    public void setThrottle(double input) {
        m_userInput = input;
        setSetpoint(input * kMaxVelocityTurnsPerSec * kWheelDiameterMeters);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("velocity m per s", () -> m_velocityMetersPerSec, null);
        builder.addDoubleProperty("motor state", this::motorState, null);
        builder.addDoubleProperty("setpoint m per s", this::getSetpoint, null);
        builder.addDoubleProperty("distance meters", this::getDistance, null);
        builder.addDoubleProperty("accel error m per s per s", this::getGetAccelerationError, null);
        builder.addDoubleProperty("velocity error m per s", this::getGetVelocityError, null);
        builder.addDoubleProperty("current dt us", () -> m_currentDt, null);
        builder.addDoubleProperty("current dx m", () -> m_currentDx, null);
        builder.addDoubleProperty("controller output", () -> m_controllerOutput, null);
        builder.addDoubleProperty("feed forward output", () -> m_feedForwardOutput, null);
        builder.addDoubleProperty("user input", () -> m_userInput, null);
        builder.addBooleanProperty("at setpoint", () -> getController().atSetpoint(), null);
    }

    // methods below are just for logging.

    /**
     * we actually give the pid velocity so asking it for position error yields
     * velocity error in meters per sec
     */
    public double getGetVelocityError() {
        return getController().getPositionError();
    }

    /**
     * we actually give the pid velocity so asking it for velocity error yields
     * acceleration error in meters per sec per sec
     */
    public double getGetAccelerationError() {
        return getController().getVelocityError();
    }

    /**
     * Motor units [-1, 1]
     */
    public double motorState() {
        return m_motor.get();
    }

    /**
     * Setpoint in meters per second
     */
    public double getSetpoint() {
        return getController().getSetpoint();
    }

}
