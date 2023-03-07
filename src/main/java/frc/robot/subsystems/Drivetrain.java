// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.math.geometry.Rotation2d;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANVenom m_frontLeft;
  private final CANVenom m_frontRight;
  private final CANVenom m_backLeft;
  private final CANVenom m_backRight;
  
  private final MecanumDrive m_drive;

  private final DoubleSolenoid m_butterfly;

  private Rotation2d rotation2d;

  // AHRS provides access to the NAVX sensors (gyrometers, inertial navigation, etc)
  // private AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  public Drivetrain() {
    m_frontLeft = new CANVenom(0);
    m_frontRight = new CANVenom(1);
    m_backLeft = new CANVenom(2);
    m_backRight = new CANVenom(3);

    m_frontRight.setInverted(true);
    m_backRight.setInverted(true);
    setBrake();
    m_drive = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);
    m_butterfly = new DoubleSolenoid(6, PneumaticsModuleType.CTREPCM, 0, 3);
  }

  public void setBrake() {
    m_frontLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_frontRight.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_backLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_backRight.setBrakeCoastMode(BrakeCoastMode.Brake);
  }

  public void pushButterfly() {
    m_butterfly.set(Value.kForward);
  }

  public void retractButterfly() {
    m_butterfly.set(Value.kReverse);
  }

  // public double getGyroAngle() {
  //   return m_gyro.getAngle();
  // }

  // public double getGyroPitch() {
  //   return m_gyro.getPitch();
  // }

  // public double getGyroRoll() {
  //   return m_gyro.getRoll();
  // }

  // public void resetGyro() {
  //   m_gyro.reset();
  // }
  

  public void drive(double xSpeed, double ySpeed, double rot) { 
    // rotation2d = Rotation2d.fromDegrees(m_gyro.getAngle());
    m_drive.driveCartesian(-xSpeed, -ySpeed, -rot);

    // m_drive.driveCartesian(forward, strafe, rotation, m_gyro.getRotation2d());
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
