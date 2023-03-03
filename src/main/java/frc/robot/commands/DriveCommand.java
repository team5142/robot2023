// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.subsystems;
import com.playingwithfusion.CANVenom;
import frc.robot.subsystems.MecaDrive;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */

  private CANVenom m_frontLeftMotor;
  private CANVenom m_frontRightMotor;
  private CANVenom m_backLeftMotor;
  private CANVenom m_backRightMotor;

  public DriveCommand() {

    addRequirements(subsystems.mecaDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_frontLeftMotor = new CANVenom(0);
    m_frontRightMotor = new CANVenom(1);
    m_backLeftMotor = new CANVenom(2);
    m_backRightMotor = new CANVenom(3);

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
