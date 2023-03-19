// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swivel;

public class ManSwivelUp extends CommandBase {
  /** Creates a new SwivelUpMan. */
  private final Swivel m_swivel;

  public ManSwivelUp(Swivel swiv) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swivel = swiv;
    addRequirements(swiv);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double power = -RobotContainer.operator.getY();
    System.out.println(power);
    m_swivel.swivelUpPower(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = -RobotContainer.operator.getY();
    System.out.println(power);

    if (power < ArmConstants.armHoldingPowerUp) {

      power = 0;
    }
    m_swivel.swivelUpPower(power);
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
