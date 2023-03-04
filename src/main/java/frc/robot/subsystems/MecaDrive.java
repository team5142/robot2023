// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C.Port;
import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.CANVenom;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

/** Represents a mecanum drive style drivetrain. */
public class MecaDrive extends SubsystemBase {
//AHRS provides access to the NAVX sensors (gyrometers, inertial navigation, etc)
  public AHRS m_gyro = new AHRS(Port.kMXP);

  Rotation2d rotation2d;
//Creates Mecanum Drive
  MecanumDrive MecaDrive;
//All motor controllers for the VENOM motors used in the drivetrain
  private CANVenom m_frontLeftMotor;
  private CANVenom m_frontRightMotor;
  private CANVenom m_backLeftMotor;
  private CANVenom m_backRightMotor;

//All motor encoders for the VENOM motors used in the drivetrain
  private final Encoder m_frontLeftEncoder = new Encoder(0, 1);
  private final Encoder m_frontRightEncoder = new Encoder(2, 3);
  private final Encoder m_backLeftEncoder = new Encoder(4, 5);
  private final Encoder m_backRightEncoder = new Encoder(6, 7);

//  Constructs a MecanumDrive and resets the gyro.
  public MecaDrive() {
    m_gyro.reset();
    m_frontLeftMotor = new CANVenom(0);
    m_frontRightMotor = new CANVenom(1);
    m_backLeftMotor = new CANVenom(2);
    m_backRightMotor = new CANVenom(3);
    //Inverts the FR motor
    m_frontRightMotor.setInverted(true);
//Inverts the BR motor
    m_backRightMotor.setInverted(true);
    MecaDrive = new MecanumDrive(m_frontLeftMotor, m_backLeftMotor, m_frontRightMotor, m_backRightMotor);
  }

  public double getGyroAngle(){
    return m_gyro.getAngle();
  }

  public double getGyroPitch(){
    return m_gyro.getPitch();
  }

  public double getGyroRoll(){
    return m_gyro.getRoll();
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

  public void SetBrake(){
    m_frontLeftMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_frontRightMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_backLeftMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_backRightMotor.setBrakeCoastMode(BrakeCoastMode.Brake);
  }
  
  public void setCoast(){
    m_frontLeftMotor.setBrakeCoastMode(BrakeCoastMode.Coast);
    m_frontRightMotor.setBrakeCoastMode(BrakeCoastMode.Coast);
    m_backLeftMotor.setBrakeCoastMode(BrakeCoastMode.Coast);
    m_backRightMotor.setBrakeCoastMode(BrakeCoastMode.Coast);
  }
  

//Resets Gyro
  public void resetGyro(){
    m_gyro.reset();
  }
  /**
   * Method to drive the robot using joystick info.
   */
  public void mecaDrive(double forward, double strafe, double rotation) {
    rotation2d = Rotation2d.fromDegrees(m_gyro.getAngle());

    MecaDrive.driveCartesian(forward, strafe, rotation, m_gyro.getRotation2d());
}
  @Override
  public void periodic() {
  // This method will be called once per scheduler run
}
}
