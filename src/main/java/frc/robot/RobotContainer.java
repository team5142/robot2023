// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutoDriveForward;
import frc.robot.commands.ManElevatorDown;
import frc.robot.commands.ManElevatorUp;
import frc.robot.commands.ManSwivelDown;
import frc.robot.commands.ManSwivelUp;
import frc.robot.commands.ManTelescopeIn;
import frc.robot.commands.ManTelescopeOut;
import frc.robot.commands.ToggleButterfly;
import frc.robot.commands.ToggleClawPiston;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swivel;
import frc.robot.subsystems.Telescope;

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
  private final Telescope m_tele = new Telescope();
  private final Claw m_claw = new Claw();

  private final ParallelCommandGroup m_liftExtend = new ParallelCommandGroup(new ManSwivelUp(m_swivel), new ManTelescopeOut(m_tele));

  private static Joystick driver = new Joystick(0);
  private static Joystick secDriver = new Joystick(1);
  public static Joystick operator = new Joystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_drive.setDefaultCommand(
        new RunCommand(
            () -> m_drive.drive(driver.getRawAxis(1), driver.getRawAxis(0), secDriver.getRawAxis(0)),
            m_drive));
  }

  private void configureBindings() {
    final JoystickButton driverRightThumb = new JoystickButton(secDriver, 2);
    final JoystickButton rightTrig = new JoystickButton(operator, 8);
    final JoystickButton rightBut = new JoystickButton(operator, 6);
    final POVButton dpadUp = new POVButton(operator, 0);
    final POVButton dpadDown = new POVButton(operator, 180);
    final POVButton dpadRight = new POVButton(operator, 90);
    final POVButton dpadLeft = new POVButton(operator, 270);
    final JoystickButton aButton = new JoystickButton(operator, 2);
    final JoystickButton bButton = new JoystickButton(operator, 3);

    driverRightThumb.onTrue(new ToggleButterfly(m_drive));
    rightTrig.whileTrue(new ManElevatorUp(m_elevator));
    rightBut.whileTrue(new ManElevatorDown(m_elevator));
    dpadUp.whileTrue(new ManSwivelUp(m_swivel));
    // dpadUp.whileTrue(m_liftExtend);
    dpadDown.whileTrue(new ManSwivelDown(m_swivel));
    dpadRight.whileTrue(new ManTelescopeOut(m_tele));
    dpadLeft.whileTrue(new ManTelescopeIn(m_tele));
    bButton.onTrue(new ToggleClawPiston(m_claw));
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new AutoDriveForward(m_drive);
  }
}
