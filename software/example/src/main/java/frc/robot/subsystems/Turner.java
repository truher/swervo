// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.motorcontrol.Parallax360;

/**
 * Turns the swerve module.
 * 
 * Measures turns in NWU radians, front of the robot is zero, +/- PI.
 * 
 * Note because of the gear reduction the modules need to be set up within 1/4
 * turn of correct at startup.
 */
public class Turner extends ProfiledPIDSubsystem {
  private static final double kP = 0.5;
  private static final double kD = 0.1;
  // private static final double kMaxVelocity = 2.1;
  private static final double kMaxVelocity = 13.2; // scaled to radians
  // private static final int kMaxAcceleration = 8;
  private static final int kMaxAcceleration = 50; // scaled to radians
  // private static final double kV = 0.4;
  private static final double kV = 0.064; // scaled to radians
  private static final double kGearRatio = 4;
  //// avoids moving from 0 to 0.5 on startup
  // private static final double kInitialPosition = 0.5;
  // point ahead
  // actually fuck this
  // private static final double kInitialPosition = 0.0;
  private static final double kDtSec = 0.02;

  public final Parallax360 m_motor;
  public final DutyCycleEncoder m_input;

  // for logging
  private double m_feedForwardOutput;
  private double m_controllerOutput;
  private double m_positionRadians;
  private double m_velocityRadiansPerSec;
  private double m_accelerationRadiansPerSecPerSec;
  private double m_setpointAccelRadiansPerSecPerSec;
  private double m_prevSetpointVelocityRadiansPerSec;
  private double m_userInput; // [-1,1]
  private final double m_offsetInEncoderTurns;

  /**
   * @param channel              for PWM motor and duty cycle encoder.
   * @param offsetInEncoderTurns Offset is measured in sensor units, [0,1]. To
   *                             adjust the module zero in the positive
   *                             (anticlockwise) direction, reduce the offset.
   */
  public Turner(int channel, double offsetInEncoderTurns) {
    super(
        new ProfiledPIDController(kP * kGearRatio, 0, kD * kGearRatio,
            new TrapezoidProfile.Constraints(kMaxVelocity / kGearRatio, kMaxAcceleration / kGearRatio)),
        0);
    getController().enableContinuousInput(-Math.PI, Math.PI);
    getController().setTolerance(0.005, 0.005); // 1.8 degrees
    setName(String.format("Turning %d", channel));
    m_motor = new Parallax360(String.format("Turn Motor %d", channel), channel);
    m_input = new DutyCycleEncoder(channel);
    m_input.setDutyCycleRange(0.027, 0.971);
    m_input.setDistancePerRotation(2 * Math.PI / kGearRatio); // "distance" = radians
    m_offsetInEncoderTurns = offsetInEncoderTurns;
    m_input.setPositionOffset(offsetInEncoderTurns);
    SmartDashboard.putData(getName(), this);
  }

  public void initialize() {
    m_input.reset(); // erase the counter to remove quarter-turn error
    m_input.setPositionOffset(m_offsetInEncoderTurns);
    enable();
  }

  /**
   * Set the profile goal in NWU radians.
   */
  @Override
  public void setGoal(double goal) {
    super.setGoal(goal);
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    m_controllerOutput = output;
    double velocityRadiansPerSec = setpoint.velocity;
    m_setpointAccelRadiansPerSecPerSec = (velocityRadiansPerSec - m_prevSetpointVelocityRadiansPerSec) / kDtSec;
    m_prevSetpointVelocityRadiansPerSec = velocityRadiansPerSec;
    // the motor is itself velocity controlled with more-or-less max acceleration
    // between setpoints, so simple velocity control is good enough.
    m_feedForwardOutput = kV * kGearRatio * velocityRadiansPerSec;
    setMotorOutput(m_controllerOutput + m_feedForwardOutput);
  }

  /**
   * Measure, correct, wrap, invert, and return NWU radians.
   */
  @Override
  public double getMeasurement() {
    double newPosition = -1.0 * MathUtil.angleModulus(m_input.getDistance());
    double newVelocity = (newPosition - m_positionRadians) / kDtSec;
    double newAcceleration = (newVelocity - m_velocityRadiansPerSec) / kDtSec;
    m_positionRadians = newPosition;
    m_velocityRadiansPerSec = newVelocity;
    m_accelerationRadiansPerSecPerSec = newAcceleration;
    return m_positionRadians;
  }

  /**
   * Set motor output from [-1, 1]
   */
  public void setMotorOutput(double value) {
    m_motor.set(value);
  }

  //
  //
  //

  /**
   * an old form of input
   */
  // control input [-1,1]
  public void setTurnRate(double input) {
    m_userInput = input;
    setGoal(MathUtil.inputModulus(m_positionRadians + input, 0, 1));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("voltage 6v", this::getVoltage6V, null);
    builder.addDoubleProperty("current 6v", this::getCurrent6V, null);
    builder.addDoubleProperty("controller output -1 to 1", this::getControllerOutput, null);
    builder.addDoubleProperty("feed forward output -1 to 1", this::getFeedForwardOutput, null);
    builder.addDoubleProperty("motor output -1 to 1", this::getMotorOutput, null);
    builder.addDoubleProperty("goal position nwu radians", this::getGoalPosition, null);
    builder.addDoubleProperty("goal velocity nwu radians per sec", this::getGoalVelocity, null);
    builder.addDoubleProperty("setpoint position nwu radians", this::getSetpointPosition, null);
    builder.addDoubleProperty("setpoint velocity nwu radians per sec", this::getSetpointVelocity, null);
    builder.addDoubleProperty("setpoint accel nwu radians per sec per sec", this::getSetpointAccel, null);
    builder.addDoubleProperty("position error nwu radians", this::getGetPositionError, null);
    builder.addDoubleProperty("velocity error nwu radians per sec", this::getGetVelocityError, null);
    builder.addDoubleProperty("position nwu radians", this::getPosition, null);
    builder.addDoubleProperty("velocity nwu radians per sec", this::getVelocity, null);
    builder.addDoubleProperty("acceleration nwu radians per sec per sec", this::getAcceleration, null);
    builder.addDoubleProperty("user input -1 to 1", () -> m_userInput, null);
  }

  // methods below are just for logging/dashboards

  public double getVoltage6V() {
    return RobotController.getVoltage6V();
  }

  public double getCurrent6V() {
    return RobotController.getCurrent6V();
  }

  /**
   * should be motor units [-1, 1]
   */
  public double getFeedForwardOutput() {
    return m_feedForwardOutput;
  }

  /**
   * should be motor units [-1, 1]
   */
  public double getControllerOutput() {
    return m_controllerOutput;
  }

  /**
   * NWU radians
   */
  public double getGoalPosition() {
    return getController().getGoal().position;
  }

  /**
   * NWU radians per second
   */
  public double getGoalVelocity() {
    return getController().getGoal().velocity;
  }

  /**
   * Radians per second per second
   * 
   * I log this to see if the controller is getting ahead of the motor.
   */
  public double getSetpointAccel() {
    return m_setpointAccelRadiansPerSecPerSec;
  }

  /**
   * Radians per second
   * 
   * I log this to see if the controller is getting ahead of the motor.
   */
  public double getSetpointVelocity() {
    return getController().getSetpoint().velocity;
  }

  /**
   * Get motor output from [-1, 1].
   */
  public double getMotorOutput() {
    return m_motor.get();
  }

  /**
   * Radians. What the PID thinks the difference is between the measurement and
   * the setpoint.
   */
  public double getGetPositionError() {
    return getController().getPositionError();
  }

  /**
   * Radians per second
   */
  public double getGetVelocityError() {
    return getController().getVelocityError();
  }

  /**
   * Most recent sampled NWU radians.
   */
  public double getPosition() {
    return m_positionRadians;
  }

  /**
   * * radians per sec
   */
  public double getVelocity() {
    return m_velocityRadiansPerSec;
  }

  /**
   * radians per sec per sec
   */
  public double getAcceleration() {
    return m_accelerationRadiansPerSecPerSec;
  }

  /**
   * what the trapezoid profile is trying to get the pid to do.
   */
  public double getSetpointPosition() {
    return getController().getSetpoint().position;
  }
}
