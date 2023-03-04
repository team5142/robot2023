// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Import necessary WPILib and CTRE libraries
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmTelescope extends SubsystemBase {
  // Creates a new ArmTelescope

  // Declare class variables
  private final TalonSRX telescopeMotor; // TalonSRX motor controller for the telescope arm
  private final Encoder encoder; // Encoder to track the arm's position
  private final PIDController telescopePID; // PID controller for position control

  // Declare constants for PID controller and distance range
  private final double kP = 0.1;
  private final double kI = 0.0;
  private final double kD = 0.0;
  private final double kTolerance = 0.5;

  private final double kMinDistance = 0.0; // Minimum distance the arm can telescope out to
  private final double kMaxDistance =
      20.0; // Maximum distance the arm can telescope out to (in inches)

  // Constructor
  public ArmTelescope() {

    // Initialize the TalonSRX motor controller on CAN bus ID 8
    telescopeMotor = new TalonSRX(8);

    // Initialize encoder on DIO ports 9 and 10
    encoder = new Encoder(9, 10);

    // Set the distance per pulse to 1 inch
    encoder.setDistancePerPulse(1.0);

    // Set the number of samples to average for filtering
    encoder.setSamplesToAverage(5);

    // Initialize the PID controller with constants kP, kI, and kD
    telescopePID = new PIDController(kP, kI, kD);

    // Enable continuous input wrapping for the PID controller
    telescopePID.enableContinuousInput(kMinDistance, kMaxDistance);

    // Set the tolerance for the PID controller's error threshold
    telescopePID.setTolerance(kTolerance);

    // Set the range for the PID controller's integral term
    telescopePID.setIntegratorRange(-0.2, 0.2);
  }

  // Method to extend the arm to a desired distance
  public void extend(double distance) {
    // Clamp the desired distance between the minimum and maximum distance range
    double target = MathUtil.clamp(distance, kMinDistance, kMaxDistance);
    // Set the target distance for the PID controller
    telescopePID.setSetpoint(target);

    // Enable continuous input for the PID controller and set its tolerance
    telescopePID.enableContinuousInput(distance, target);
    telescopePID.setTolerance(kTolerance);

    // Calculate the output value for the motor controller using the PID controller
    double output = telescopePID.calculate(encoder.getDistance(), target);

    // Set the motor controller's output to the calculated output
    telescopeMotor.set(ControlMode.PercentOutput, output);

    // Update SmartDashboard values for debugging purposes
    SmartDashboard.putNumber("Telescope Setpoint", target);
    SmartDashboard.putNumber("Telescope Distance", encoder.getDistance());
  }

  // Method to stop the arm from moving
  public void stop() {
    // Reset the PID controller and set the motor controller's output to 0
    telescopePID.reset();
    telescopeMotor.set(ControlMode.PercentOutput, 0);
  }

  // Method to check if the arm is at its target position
  public boolean isAtTarget() {
    return telescopePID.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
