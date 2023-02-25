// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.playingwithfusion.CANVenom;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;

public class ButterflyTankDrive extends SubsystemBase {
//Creates a single solenoid that controlls all four butterfly modules, dropping/retracting the traction wheels
DoubleSolenoid ButterflySolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

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

  /** Creates a new ButterflyDrive. */
  public ButterflyTankDrive() {
  
// Gets the state of the solenoid (True/False)

/* WILL FINISH LATER
public boolean ButterflyActuationState(){
  return ButterflySolenoid.get();}
*/
  
  

  }

=======

import java.lang.ModuleLayer.Controller;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;


public class ButterflyTankDrive extends SubsystemBase {
  
    private AHRS navx;
    private XboxController controller;
    private DifferentialDrive ButterflyDrive;
    
    private static final double kOffBalanceAngleThresholdDegrees = 10;
    private static final double kOnBalanceAngleThresholdDegrees = 5;
    private boolean autoBalanceMode;
    
    public void operatorControl() {
        
        double leftSpeed = 0;
        double rightSpeed = 0;
        
        double xAxisRate = controller.getLeftX();
        double yAxisRate = controller.getLeftY();
        double pitchAngleDegrees = navx.getPitch();
        
        if (!autoBalanceMode && 
            Math.abs(pitchAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees)) {
          autoBalanceMode = true;
        } else if (autoBalanceMode && 
                   Math.abs(pitchAngleDegrees) <= Math.abs(kOnBalanceAngleThresholdDegrees)) {
          autoBalanceMode = false;
        }
        
        double deadband = 0.1; // adjust deadband value as needed
        
        if (autoBalanceMode) {
          double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
          double pitchCorrection = Math.sin(pitchAngleRadians);
          
          if (Math.abs(pitchCorrection) > deadband) {
            leftSpeed = -pitchCorrection;
            rightSpeed = -pitchCorrection;
          }
        } else {
          leftSpeed = yAxisRate + xAxisRate;
          rightSpeed = yAxisRate - xAxisRate;
        }
        
        ButterflyDrive.tankDrive(leftSpeed, rightSpeed);
        Timer.delay(0.005); // wait for a motor update time
      }
>>>>>>> 83a5b83 (Update)

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}