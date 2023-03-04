 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.CANVenom;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  // Define constants for the elevator
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kF = 0.0;
  private static final double kTolerance = 0.5;
  
  private static final int kElevatorEncoderCountsPerInch = 4096;
  private static final int kElevatorMaxEncoderCounts = 10000;
  private static final int kElevatorMinEncoderCounts = 0;

  // Define the elevator motor and encoder
  private CANVenom elevatorMotor;
  private Encoder elevatorEncoder;

  // Define the PID controller for the elevator
  private PIDController elevatorPID;

  // Constructor for the elevator subsystem
  public Elevator() {
     // Create the elevator motor and encoder objects
    elevatorMotor = new CANVenom(4);
    elevatorEncoder = new Encoder(11,12);

     // Set the encoder distance per pulse
     elevatorEncoder.setDistancePerPulse(1.0 / kElevatorEncoderCountsPerInch);

     // Set the PID controller for the elevator
    elevatorPID = new PIDController(kP, kI, kD, kF);
    elevatorPID.setTolerance(kTolerance);
  }

  // Method to extend the elevator to a specific setpoint in inches
  public void setElevatorPosition(double positionInches) {

    // Limit the setpoint to the allowable range
    positionInches = Math.max(Math.min(positionInches, kElevatorMaxEncoderCounts / kElevatorEncoderCountsPerInch), kElevatorMinEncoderCounts / kElevatorEncoderCountsPerInch);
    
    // Convert the setpoint to encoder counts
    int positionEncoderCounts = (int) (positionInches * kElevatorEncoderCountsPerInch);
    
    // Set the setpoint for the PID controller
    elevatorPID.setSetpoint(positionEncoderCounts);
    
    // Set the elevator motor speed based on the PID controller output
    double motorSpeed = elevatorPID.calculate(elevatorEncoder.getDistance());
    elevatorMotor.set(motorSpeed);
  }

  // Method to stop the elevator
  public void stopElevator() {
    elevatorMotor.set(0.0);
  }
  
  // Method to check if the elevator is at the setpoint
  public boolean atSetpoint() {
    return elevatorPID.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
