// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swivel;

public class ManSwivelDown extends CommandBase {
  /** Creates a new SwivelDownMan. */
  private final Swivel m_swivel;

  public ManSwivelDown(Swivel swiv) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swivel = swiv;
    addRequirements(swiv);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swivel.swivelDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swivel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
