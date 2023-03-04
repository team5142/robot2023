// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANVenom m_frontLeft;
  private final CANVenom m_frontRight;
  private final CANVenom m_backLeft;
  private final CANVenom m_backRight;
  
  private final MecanumDrive m_drive;

  public Drivetrain() {
    m_frontLeft = new CANVenom(0);
    m_frontRight = new CANVenom(1);
    m_backLeft = new CANVenom(2);
    m_backRight = new CANVenom(3);

    m_drive = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);
  }

  public void drive(double forward, double rot) {
    m_drive.driveCartesian(forward, rot, 0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
