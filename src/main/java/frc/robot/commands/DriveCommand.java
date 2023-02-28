// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.subsystems;

public class DriveCommand extends CommandBase {
  /** Creates a new XDrive. */
  public DriveCommand() {

    addRequirements(subsystems.mecaDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystems.mecaDrive.SetBrake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = RobotContainer.driverController.getLeftY();
    double strafe = RobotContainer.driverController.getLeftX();
    double rotation = RobotContainer.driverController.getRightX();

    MathUtil.applyDeadband(forward, 0.02);
    MathUtil.applyDeadband(strafe, 0.02);
    MathUtil.applyDeadband(rotation, 0.02);

    subsystems.mecaDrive.mecaDrive(forward, strafe, rotation);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
