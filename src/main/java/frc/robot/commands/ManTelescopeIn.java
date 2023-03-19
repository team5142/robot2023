// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Telescope;

public class ManTelescopeIn extends CommandBase {
  /** Creates a new TelescopeInMan. */
  private final Telescope m_telescope;

  public ManTelescopeIn(Telescope tele) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_telescope = tele;
    addRequirements(tele);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_telescope.TelescopeIn();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_telescope.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
