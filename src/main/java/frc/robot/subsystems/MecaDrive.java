// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a mecanum drive style drivetrain. */
public class MecaDrive extends SubsystemBase {
  // AHRS provides access to the NAVX sensors (gyrometers, inertial navigation, etc)
  private AHRS m_gyro = new AHRS(Port.kMXP);

  private Rotation2d rotation2d;
  // Creates Mecanum Drive
  private MecanumDrive MecaDrive;
  private CANVenom m_frontLeftMotor;

  //  Constructs a MecanumDrive and resets the gyro.
  public MecaDrive() {
    m_frontLeftMotor = new CANVenom(0);

    m_gyro.reset();

    // MecaDrive = new MecanumDrive(m_frontLeftMotor, m_backLeftMotor, m_frontRightMotor,
    // m_backRightMotor);
  }

  public double getGyroAngle() {
    return m_gyro.getAngle();
  }

  public double getGyroPitch() {
    return m_gyro.getPitch();
  }

  public double getGyroRoll() {
    return m_gyro.getRoll();
  }
  /**
   * Returns the current state of the drivetrain.
   *
   * @return The current state of the drivetrain.
   */
  // public MecanumDriveWheelSpeeds getCurrentState() {
  //   return new MecanumDriveWheelSpeeds(
  //     //returns the speeds of the VENOMs according to the encoders
  //       m_frontLeftEncoder.getRate(),
  //       m_frontRightEncoder.getRate(),
  //       m_backLeftEncoder.getRate(),
  //       m_backRightEncoder.getRate());
  // }
  /**
   * Returns the current distances measured by the drivetrain.
   *
   * @return The current distances measured by the drivetrain.
   */
  // public MecanumDriveWheelPositions getCurrentDistances() {
  //   return new MecanumDriveWheelPositions(
  //       m_frontLeftEncoder.getDistance(),
  //       m_frontRightEncoder.getDistance(),
  //       m_backLeftEncoder.getDistance(),
  //       m_backRightEncoder.getDistance());
  // }

  public void SetBrake() {
    m_frontLeftMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
    // m_frontRightMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
    // m_backLeftMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
    // m_backRightMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
  }

  public void setCoast() {
    m_frontLeftMotor.setBrakeCoastMode(BrakeCoastMode.Coast);
    // m_frontRightMotor.setBrakeCoastMode(BrakeCoastMode.Coast);
    // m_backLeftMotor.setBrakeCoastMode(BrakeCoastMode.Coast);
    // m_backRightMotor.setBrakeCoastMode(BrakeCoastMode.Coast);
  }

  // Resets Gyro
  public void resetGyro() {
    m_gyro.reset();
  }
  /** Method to drive the robot using joystick info. */
  public void mecaDrive(double forward, double strafe, double rotation) {
    rotation2d = Rotation2d.fromDegrees(m_gyro.getAngle());

    MecaDrive.driveCartesian(forward, strafe, rotation, m_gyro.getRotation2d());
  }
}
