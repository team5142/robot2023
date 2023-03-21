// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Import necessary WPILib and CTRE libraries
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
  /** Creates a new Telescope. */

  // All reference points (For placement/collection)
  private double shortTelescopeLength = 0; // Telescope length for short setpoint

  private double midTelescopeLength = 0; // Telescope length for mid setpoint

  private double longTelescopeLength = 0; // Telescope length for long setpoint

  // Declare class variables
  private final TalonSRX m_telescope; // TalonSRX motor controller for the telescope arm

  private final Encoder m_encoder; // Encoder for the RedLine motor

  private final PIDController m_controller; // PID controller for position control

  public double m_targetPosition; // Target position for the motor

  double dist = 0.5 * 3.14 / 1024; // ft per pulse

  // Declare constants for PID controller and distance range
  private final double kP = 0.1;
  private final double kI = 0.0;
  private final double kD = 0.0;

  public Telescope() {
    m_telescope = new TalonSRX(10); // Telescoping motor CAN ID
    TalonSRXConfiguration configuration = new TalonSRXConfiguration(); // Man idk what this does

    m_encoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
    m_encoder.setDistancePerPulse(dist);

    m_telescope.setNeutralMode(NeutralMode.Brake); // Initialize the motor as braked
    m_telescope.configAllSettings(configuration); // Man idk what this does

    m_controller = new PIDController(0.1, 0, 0); // PID Control (Will change later)
  }

  public void TelescopeOut() {
    m_telescope.set(ControlMode.PercentOutput, 0.5); // Sets motor output to +50%
  }

  public void TelescopeIn() {
    m_telescope.set(ControlMode.PercentOutput, -0.5); // Sets motor output to -50%
  }

  public void stop() {
    m_telescope.set(ControlMode.Position, 0); // Sets motor output to 0
  }

  public void TelescopeShort() {
    m_targetPosition = shortTelescopeLength; // Set the target position to the ground angle
  }

  public void TelescopeMid() {
    m_targetPosition = midTelescopeLength; // Set the target position to the single substation
    // angle
  }

  public void TelescopeLong() {
    m_targetPosition = longTelescopeLength; // Set the target position to be parallel to the ground
  }

  public void initialize() {
    m_encoder.setDistancePerPulse(
        m_controller.calculate(
            m_encoder.getDistance(),
            0)); // Initializes the encoder at a set position (Needs some tweaking)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
