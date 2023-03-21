// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swivel extends SubsystemBase {

  /** Creates a new Swivel. */

  // All reference angles (For placement/collection)
  private double groundSwivelAngle = 0; // Swivel angle for ground collection

  private double singleSubSwivelAngle = 0; // Swivel angle for single substation collection

  private double straightSwivelAngle = 0; // Swivel angle for double substation collection

  // The TalonSRX motor controller that controls the swivel motion.
  private final CANSparkMax m_swivel;
  private final RelativeEncoder m_encoder;
  private final PIDController m_controller;

  // Target position for the motor
  public double m_targetPosition;

  public Swivel() {
    m_swivel = new CANSparkMax(8, MotorType.kBrushless); // Swivel Motor CAN ID
    m_encoder = m_swivel.getEncoder();

    m_swivel.setIdleMode(IdleMode.kBrake); // Initialize the motor as braked

    m_controller = new PIDController(0.2, 0, 0.04); // PID Control (Requires more testing)

    m_targetPosition = m_encoder.getPosition();
  }

  public void swivelUp() {
    m_swivel.set(0.2); // Sets motor speed to +20%
  }

  public void swivelUpPower(double power) {
    m_swivel.set(power); // Sets a constant holding power (To resist gravity)
  }

  public void swivelDown() {
    m_swivel.set(-0.2); // Sets motor speed to -40%
  }

  public void stop() {
    m_swivel.set(0); // Sets motor speed to 0
  }

  public void swivelGround() {
    m_targetPosition = groundSwivelAngle; // Set the target position to the ground angle
  }

  public void swivelSingleSubstationCollect() {
    m_targetPosition = singleSubSwivelAngle; // Set the target position to the single substation
    // angle
  }

  public void swivelStraight() {
    m_targetPosition = straightSwivelAngle; // Set the target position to be parallel to the ground
  }

  public double getEncoder() {
    return m_encoder.getPosition(); // Returns the absolute position of the CANcoder
  }

  public void initialize() {
    m_encoder.setPosition(
        m_controller.calculate(
            m_encoder.getPosition(),
            0)); // Initializes the encoder at a set position (Needs some tweaking)
  }

  public void setPosition(double position) {

    // Set the target position to the given position
    m_targetPosition = position;

    // Set the setpoint of the PID controller to the target position
    m_controller.setSetpoint(m_targetPosition);

    // Reset the velocity of the motor controller to 0
    m_swivel.getPIDController().setReference(0, ControlType.kVelocity);

    // Set the reference velocity of the motor controller to the output of the PID controller
    m_swivel
        .getPIDController()
        .setReference(
            m_controller.calculate(m_encoder.getPosition(), m_targetPosition),
            ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SwivelEnc", getEncoder()); // For testing
    // This method will be called once per scheduler run
  }
}
