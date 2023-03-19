// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutoDriveForward;
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

  // private final ParallelCommandGroup m_liftExtend = new ParallelCommandGroup(new
  // ManSwivelUp(m_swivel), new ManTelescopeOut(m_tele));

  private static Joystick driver = new Joystick(0);
  private static Joystick Driver2 = new Joystick(1);
  public static Joystick operator = new Joystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_drive.setDefaultCommand(
        new RunCommand(
            () -> m_drive.drive(driver.getRawAxis(1), driver.getRawAxis(0), Driver2.getRawAxis(0)),
            m_drive));
    /*
    m_elevator.setDefaultCommand(
        new RunCommand(
          () -> m_elevator.manualUp(operator.getRawAxis(1)),// Y-axis controls Up/Down movement
            m_elevator));
            */
  }

  private void configureBindings() {

    // Driver Buttons
    final JoystickButton driverRightThumb = new JoystickButton(Driver2, 2);

    // Operator Buttons
    final JoystickButton triggerButton = new JoystickButton(operator, 1); // Flips claw in/out
    final JoystickButton thumbButton = new JoystickButton(operator, 2); //
    final JoystickButton button3 = new JoystickButton(operator, 3); //
    final JoystickButton button4 = new JoystickButton(operator, 4); //
    final JoystickButton button5 = new JoystickButton(operator, 5); //
    final JoystickButton button6 = new JoystickButton(operator, 6); //
    final JoystickButton button7 = new JoystickButton(operator, 7); // Place High Cone
    final JoystickButton button8 = new JoystickButton(operator, 8); // Place High Cube
    final JoystickButton button9 = new JoystickButton(operator, 9); // Place Mid Cone
    final JoystickButton button10 = new JoystickButton(operator, 10); // Place Mid Cube
    final JoystickButton button11 = new JoystickButton(operator, 11); // Grab from Single Substation
    final JoystickButton button12 = new JoystickButton(operator, 12); // Grab from Double Substation
    final POVButton povPadUp = new POVButton(operator, 0); // Swivel Up
    final POVButton povPadDown = new POVButton(operator, 180); // Swivel Down
    final POVButton povPadRight = new POVButton(operator, 90); // Telescope Out
    final POVButton povPadLeft = new POVButton(operator, 270); // Telescope In

    // Driver Command Bindings
    driverRightThumb.onTrue(new ToggleButterfly(m_drive));

    // Operator Command Bindings
    // rightTrig.whileTrue(new ManElevatorUp(m_elevator));
    // rightBut.whileTrue(new ManElevatorDown(m_elevator));
    povPadUp.whileTrue(new ManSwivelUp(m_swivel));
    povPadDown.whileTrue(new ManSwivelDown(m_swivel));
    povPadRight.whileTrue(new ManTelescopeOut(m_tele));
    povPadLeft.whileTrue(new ManTelescopeIn(m_tele));
    triggerButton.onTrue(new ToggleClawPiston(m_claw));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new AutoDriveForward(m_drive);
  }
}
