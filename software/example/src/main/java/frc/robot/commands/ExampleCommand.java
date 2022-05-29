// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExampleCommand extends CommandBase {
  private static final double kMaxSpeedMetersPerSec = 0.54;
  private static final double kRobotRadiusMeters = 0.1613;
  private final XboxController m_input;
  private final Drivetrain m_subsystem;

  private double m_steer_input;
  private double m_drive_input;

  public ExampleCommand(XboxController input, Drivetrain subsystem) {
    m_input = input;
    m_subsystem = subsystem;
    addRequirements(subsystem);
    SmartDashboard.putData("Example Command", this);
  }

  @Override
  public void execute() {
    // test mode, steering response to goal step
    // m_subsystem.setTurningGoal(m_input.getAButton()?0.25:0.7);

    // normal mode, right Y drives, left X steers
    // m_steer_input = (-1 * m_input.getLeftX() + 1) / 2;
    // m_drive_input = -1 * m_input.getRightY();
    // m_subsystem.setTurnRate(m_steer_input);
    // m_subsystem.setThrottle(m_drive_input);

    // drone mode, right X/Y is everything
    // double xInput = -1 * m_input.getRightX(); // [-1,1]
    // double yInput = m_input.getRightY(); // [-1,1]
    // m_subsystem.setTurnGoal(Units.radiansToRotations(new Rotation2d(xInput,
    // yInput).getRadians()));
    // m_subsystem.setThrottle(Math.hypot(xInput, yInput));

    // m_subsystem.keepWheelsPointingNorth();

    // every axis of every joystick is backwards.
    m_subsystem.drive(
        -1.0 * kMaxSpeedMetersPerSec * m_input.getRightY(),
        -1.0 * kMaxSpeedMetersPerSec * m_input.getRightX(),
        -1.0 * kMaxSpeedMetersPerSec / kRobotRadiusMeters * m_input.getLeftX());
  }

  public double getSteerInput() {
    return m_steer_input;
  }

  public double getDriveInput() {
    return m_drive_input;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("steer input", this::getSteerInput, null);
    builder.addDoubleProperty("drive input", this::getDriveInput, null);
  }

  @Override
  public void initialize() {
    m_subsystem.initialize();
  }
}
