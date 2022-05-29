// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final XboxController m_driverController;
  private final Drivetrain m_subsystemGroup;
  private final Command m_teleopCommand;

  public RobotContainer() {
    // turn off logging for now
    // DataLogManager.start();
    m_driverController = new XboxController(1);
    m_subsystemGroup = new Drivetrain();
    m_teleopCommand = new ExampleCommand(m_driverController, m_subsystemGroup);
  }

  public Command getTeleopCommand() {
    return m_teleopCommand;
  }

  //public void runTest() {
    //boolean button = m_driverController.getAButton();
    //m_subsystemGroup.runTest(button?0.3:-0.3);
    //m_subsystemGroup.runTest2(button);
    //m_subsystemGroup.runTest3(button);
    //m_subsystemGroup.runTest4();
  //}
}
