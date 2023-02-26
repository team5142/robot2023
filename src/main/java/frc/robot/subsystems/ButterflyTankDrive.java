// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}