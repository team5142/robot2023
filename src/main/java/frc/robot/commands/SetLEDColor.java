// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ledStrip;

public class SetLEDColor extends CommandBase {
  /** Creates a new SetLEDColor. */
  private final ledStrip m_ledStrip;
  private final Color m_color = new Color(255, 255, 0);

  public SetLEDColor(ledStrip ledStrip) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ledStrip = ledStrip;
    addRequirements(m_ledStrip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ledStrip.setColor(m_color);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
