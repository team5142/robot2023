// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.MainDrive;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain m_drive = new Drivetrain();
  // private final MainDrive m_mec = new MainDrive(m_drive, null, null, null)

  public static XboxController driverController = new XboxController(0);
  public static Joystick operatorController = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_drive.setDefaultCommand(
      new RunCommand(
        () ->
          m_drive.drive(
            -driverController.getLeftY(),
            -driverController.getRightX(), 
            -driverController.getLeftX()),
        m_drive));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
