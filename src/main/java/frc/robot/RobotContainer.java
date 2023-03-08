// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorDownMan;
import frc.robot.commands.ElevatorUpMan;
import frc.robot.commands.PushButterfly;
import frc.robot.commands.RetractButterfly;
import frc.robot.commands.SwivelDownMan;
import frc.robot.commands.SwivelUpMan;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swivel;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain m_drive = new Drivetrain();
  private final Elevator m_elevator = new Elevator();
  private final Swivel m_swivel = new Swivel();

  private static Joystick driver = new Joystick(0);
  public static Joystick operatorController = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_drive.setDefaultCommand(
        new RunCommand(
            () -> m_drive.drive(driver.getRawAxis(1), driver.getRawAxis(0), driver.getRawAxis(2)),
            m_drive));
  }

  private void configureBindings() {
    final JoystickButton yButton = new JoystickButton(driver, 4);
    final JoystickButton aButton = new JoystickButton(driver, 2);
    final JoystickButton rTrig = new JoystickButton(driver, 8);
    final JoystickButton rButt = new JoystickButton(driver, 6);
    final JoystickButton lTrig = new JoystickButton(driver, 7);
    final JoystickButton lButt = new JoystickButton(driver, 5);

    aButton.onTrue(new PushButterfly(m_drive));
    yButton.onTrue(new RetractButterfly(m_drive));
    rTrig.whileTrue(new ElevatorUpMan(m_elevator));
    rButt.whileTrue(new ElevatorDownMan(m_elevator));
    lTrig.whileTrue(new SwivelUpMan(m_swivel));
    lButt.whileTrue(new SwivelDownMan(m_swivel));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
