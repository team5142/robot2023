// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ButterflyTankDrive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MecaDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  public static class subsystems {
    public final static MecaDrive mecaDrive = new MecaDrive();
    public final static ButterflyTankDrive butterflyTankDrive = new ButterflyTankDrive();
    public final static Arm arm = new Arm();
    public final static Claw claw = new Claw();
    public final static Elevator elevator = new Elevator();
  }

  public static class commands {

    public final static DriveCommand DriveCommand = new DriveCommand();

  }
  
  public static XboxController driverController = new XboxController(0);
  public static XboxController operatorController = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
