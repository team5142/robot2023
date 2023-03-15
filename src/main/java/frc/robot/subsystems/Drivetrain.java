// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANVenom m_frontLeft;
  private final CANVenom m_frontRight;
  private final CANVenom m_backLeft;
  private final CANVenom m_backRight;

  private final MecanumDrive m_drive;

  private final DoubleSolenoid m_butterfly;

  private final AHRS m_navX;

  private Boolean m_isButterfly;

  public Drivetrain() {
    m_frontLeft = new CANVenom(0);
    m_frontRight = new CANVenom(1);
    m_backLeft = new CANVenom(2);
    m_backRight = new CANVenom(3);

    m_frontRight.setInverted(true);
    m_backRight.setInverted(true);
    setBrake();
    m_drive = new MecanumDrive(m_frontLeft, m_backLeft, m_frontRight, m_backRight);
    m_butterfly = new DoubleSolenoid(6, PneumaticsModuleType.CTREPCM, 1, 2);
    retractButterfly();
    m_navX = new AHRS(SPI.Port.kMXP);
    m_isButterfly = false;
  }

  public void setBrake() {
    m_frontLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_frontRight.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_backLeft.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_backRight.setBrakeCoastMode(BrakeCoastMode.Brake);
  }

  private void pushButterfly() {
    m_butterfly.set(Value.kReverse);
    m_isButterfly = true;
  }
  

  public void resetEncoders() {
    m_frontLeft.resetPosition();
    m_frontRight.resetPosition();
    m_backLeft.resetPosition();
    m_backRight.resetPosition();
  }

  private void retractButterfly() {
    m_butterfly.set(Value.kForward);
    m_isButterfly = false;
  }

  public void toggleButterfly() {
    if (m_isButterfly == true) {
      retractButterfly();
    } else {
      pushButterfly();
    }
  }

  public void autobalance() {
    final double P = 0.05; // Proportional gain
    final double I = 0.0; // Integral gain
    final double D = 0.0; // Derivative gain
    final double toleranceAngle = 0.5; // Deadband tolerance in degrees
  
    double errorAngle = -m_navX.getPitch(); // Negative sign to make positive pitch angle tilt forward
    double integral = 0.0;
    double derivative = 0.0;
    double previousErrorAngle = 0.0;
  
    while (Math.abs(errorAngle) > toleranceAngle) {
      errorAngle = -m_navX.getPitch(); // Gets the pitch of the robot (Relative to zero)

      integral += errorAngle * 0.02; // Integration time step of 20 ms
      derivative = (errorAngle - previousErrorAngle) / 0.02; // Differentiation time step of 20 ms

      double output = P * errorAngle + I * integral + D * derivative;

      output = MathUtil.clamp(output, -0.2, 0.2); // Limit output to -20% to +20% speed
  
      // Set all four motors to run at the same output in mecanum drive formation (Only going forwards/backwards)
      m_drive.driveCartesian(output, 0.0, 0.0);
  
      previousErrorAngle = errorAngle; // Updates the hierarchy, running through the method again (Until balance is acheived)

      Timer.delay(0.02); // Wait 20 ms before checking again
    }
  
    // Stop motors after reaching balance
    m_drive.driveCartesian(0.0, 0.0, 0.0);
  }
  

  public void drive(double xSpeed, double ySpeed, double rot) {
    Rotation2d rotation2d = Rotation2d.fromDegrees(m_navX.getAngle());
    m_drive.driveCartesian(-xSpeed, ySpeed, rot, rotation2d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
