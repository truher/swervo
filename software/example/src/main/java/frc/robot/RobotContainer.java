// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MySwerveCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final XboxController m_driverController;
  private final Drivetrain m_drivetrain;
  private final Field2d m_field;
  private final Command m_teleopCommand;
  private final Command m_autonomousCommand;

  public RobotContainer() {
    // turn off logging for now
    // DataLogManager.start();
    m_driverController = new XboxController(1);
    m_drivetrain = new Drivetrain();
    m_field = new Field2d();
    m_teleopCommand = new ExampleCommand(m_driverController, m_drivetrain, m_field);
    // TODO: make this stop some other way.
    m_autonomousCommand = new MySwerveCommand(m_drivetrain, m_field).andThen(() -> m_drivetrain.drive(0, 0, 0));
    SmartDashboard.putData("Field", m_field);
  }

  public Command getTeleopCommand() {
    return m_teleopCommand;
  }

  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }

  public void disable() {
    m_drivetrain.zeroModules();
  }

  // public void runTest() {
  // boolean button = m_driverController.getAButton();
  // m_subsystemGroup.runTest(button?0.3:-0.3);
  // m_subsystemGroup.runTest2(button);
  // m_subsystemGroup.runTest3(button);
  // m_subsystemGroup.runTest4();
  // }
}
