// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ButterflyDriveCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSwivel;
import frc.robot.subsystems.ArmTelescope;
import frc.robot.subsystems.ButterflyPneumatics;
import frc.robot.subsystems.ButterflyTankDrive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Harvester;
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
    public final static ArmSwivel armSwivel = new ArmSwivel();
    public final static ArmTelescope armTelescope = new ArmTelescope();
    public final static Claw claw = new Claw();
    public final static ButterflyTankDrive butterflyTankDrive = new ButterflyTankDrive();
    public final static ButterflyPneumatics butterflyPneumatics = new ButterflyPneumatics();
    public final static Elevator elevator = new Elevator();
    public final static Harvester harvester = new Harvester();
    public final static MecaDrive mecaDrive = new MecaDrive();

  }

  public static class commands {

    public final static DriveCommand DriveCommand = new DriveCommand();
    public final static ButterflyDriveCommand ButterflyDriveCommand = new ButterflyDriveCommand();

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
