// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class MainDrive extends CommandBase {
  private final Drivetrain m_drive;
  private final DoubleSupplier m_x, m_y, m_z;
  /** Creates a new MainDrive. */
  public MainDrive(Drivetrain drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_x = x;
    m_y = y;
    m_z = z;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(m_x.getAsDouble(), m_y.getAsDouble(), m_z.getAsDouble());
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
