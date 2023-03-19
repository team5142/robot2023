// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotContainer;

public class ManElevatorUp extends CommandBase {
  /** Creates a new ElevatorUpMan. */
  private final Elevator m_elev;

  public ManElevatorUp(Elevator elev) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elev = elev;
    addRequirements(elev);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double ePower = -RobotContainer.operator.getY();
    System.out.println(ePower);
    m_elev.elevatorUpPower(ePower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ePower = -RobotContainer.operator.getX();
    System.out.println(ePower);

    if (ePower<ElevatorConstants.elevHoldingPowerUp) {

      ePower = 0;

    }
    m_elev.elevatorUpPower(ePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elev.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}