// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.CANVenom;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.playingwithfusion.CANVenom.BrakeCoastMode;


/** Represents a mecanum drive style drivetrain. */
public class MecaDrive extends SubsystemBase {
//AHRS provides access to the NAVX sensors (gyrometers, inertial navigation, etc)
  public AHRS gyro = new AHRS(Port.kMXP);
  Rotation2d rotation2d;
//Creates the driver controller
  XboxController controller;
//Creates Mecanum Drive
  MecanumDrive MecaDrive;
//All motor controllers for the VENOM motors used in the drivetrain
  private final MotorController m_frontLeftMotor = new CANVenom(0);
  private final MotorController m_frontRightMotor = new CANVenom(1);
  private final MotorController m_backLeftMotor = new CANVenom(2);
  private final MotorController m_backRightMotor = new CANVenom(3);

//All motor encoders for the VENOM motors used in the drivetrain
  private final Encoder m_frontLeftEncoder = new Encoder(0, 1);
  private final Encoder m_frontRightEncoder = new Encoder(2, 3);
  private final Encoder m_backLeftEncoder = new Encoder(4, 5);
  private final Encoder m_backRightEncoder = new Encoder(6, 7);

//The gyrometer
  private final AnalogGyro m_gyro = new AnalogGyro(0);
//  Constructs a MecanumDrive and resets the gyro.
  public MecaDrive() {
    m_gyro.reset();
//Inverts the FR motor
    m_frontRightMotor.setInverted(true);
//Inverts the BR motor
    m_backRightMotor.setInverted(true);
    MecaDrive = new MecanumDrive(m_frontLeftMotor, m_backLeftMotor, m_frontRightMotor, m_backRightMotor);
  }

  public double getGyroAngle(){
    return gyro.getAngle();
  }

  public double getGyroPitch(){
    return gyro.getPitch();
  }

  public double getGyroRoll(){
    return gyro.getRoll();
  }
  /**
   * Returns the current state of the drivetrain.
   *
   * @return The current state of the drivetrain.
   */
  public MecanumDriveWheelSpeeds getCurrentState() {
    return new MecanumDriveWheelSpeeds(
      //returns the speeds of the VENOMs according to the encoders
        m_frontLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_backLeftEncoder.getRate(),
        m_backRightEncoder.getRate());
  }
  /**
   * Returns the current distances measured by the drivetrain.
   *
   * @return The current distances measured by the drivetrain.
   */
  public MecanumDriveWheelPositions getCurrentDistances() {
    return new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(),
        m_frontRightEncoder.getDistance(),
        m_backLeftEncoder.getDistance(),
        m_backRightEncoder.getDistance());
  }
/*
  public void SetBrake(){
    m_frontLeftMotor.setBrake(BrakeCoastMode.Brake);
    m_frontRightMotor.setBrake(BrakeCoastMode.Brake);
    m_backLeftMotor.setBrake(BrakeCoastMode.Brake);
    m_backRightMotor.setBrake(BrakeCoastMode.Brake);
  }
  
  public void setCoast(){
    m_frontLeftMotor.BrakeCoastMode(BrakeCoastMode.Coast);
    m_frontRightMotor.setCoast(BrakeCoastMode.Coast);
    m_backLeftMotor.setCoast(BrakeCoastMode.Coast);
    m_backRightMotor.setCoast(BrakeCoastMode.Coast);
  }
*/
  public void resetGyro(){
    gyro.reset();
  }

  /**
   * Set the desired speeds for each wheel.
   *
   * @param speeds The desired wheel speeds.
   */

  /**
   * Method to drive the robot using joystick info.
   */
  public void mecaDrive(double forward, double strafe, double rotation) {
    rotation2d = Rotation2d.fromDegrees(gyro.getAngle());

      MecaDrive.driveCartesian(forward, strafe, rotation, gyro.getRotation2d());
}
}
