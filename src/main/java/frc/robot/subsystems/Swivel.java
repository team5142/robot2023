// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swivel extends SubsystemBase {

  private WPI_TalonSRX armMotorController;

  /** Creates a new Swivel. */

  // All Cone/Cube reference angles (For placement)
  private double groundCollectSwivelAngle = 0; // Swivel angle for ground collection 
  private double substationCollectSwivelAngle = 0; // Swivel angle for double substation collection
  private double lowSwivelAngle = 0; // Swivel Angle for low hub (Cone/Cube)
  public double midSwivelAngle = 130; // Swivel Angle for mid hub (Cone/Cube)
  private double highSwivelAngle = 0; // Swivel Angle for high hub (Cone/Cube)

  // The TalonSRX motor controller that controls the swivel motion.
  private final TalonSRX m_swivel;
  private final CANCoder m_encoder;
  private final PIDController m_controller;

  public Swivel() {
    m_swivel = new TalonSRX(8); // Swivel Motor CAN ID
    m_encoder = new CANCoder(11); // CANCoder CAN ID

    m_swivel.setNeutralMode(NeutralMode.Brake); // Initialize the motor as braked
    m_encoder.configFactoryDefault(); // Man Idk what this does

    m_controller = new PIDController(0.2, 0, 0.04); // PID Control (Requires more testing)
  }

  public void swivelUp() {
    m_swivel.set(ControlMode.PercentOutput, 0.2, DemandType.ArbitraryFeedForward, 0.2); // Sets motor speed to +20%
  }

  public void swivelUpPower(double power) {
    m_swivel.set(TalonSRXControlMode.PercentOutput, power); // Sets a constant holding power (To resist gravity)
  }

  public void swivelDown() {
    m_swivel.set(ControlMode.PercentOutput, -0.2); // Sets motor speed to -40%
  }

  public void stop() {
    m_swivel.set(ControlMode.PercentOutput, 0); // Sets motor speed to 0
  }

  public void swivelLow() {
    m_swivel.set(ControlMode.Position, lowSwivelAngle); // Swivels the arm to the set low hub angle
  }

  public void swivelMid() {
    m_swivel.set(ControlMode.PercentOutput, -m_controller.calculate(m_encoder.getAbsolutePosition(), midSwivelAngle)); // Swivels the arm to the set mid hub angle (Based on PID control)
  }

  public void swivelHigh() {
    m_swivel.set(ControlMode.Position, highSwivelAngle); // Swivels the arm to the set high hub angle
  }

  public void swivelGroundCollect() {
    m_swivel.set(ControlMode.Position, groundCollectSwivelAngle); // Swivels the arm to the set ground angle
  }

  public void swivelSubstationCollect() {
    m_swivel.set(ControlMode.Position, substationCollectSwivelAngle); // Swivels the arm to the set double substation angle
  }

  public double getEncoder() {
    return m_encoder.getAbsolutePosition(); // Returns the absolute position of the CANcoder
  }

  public void initialize() {
    m_swivel.set(ControlMode.Position, m_controller.calculate(m_encoder.getPosition(), 350)); // Initializes the encoder at a set position (Needs some tweaking)
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SwivelEnc", getEncoder()); // For testing
    // This method will be called once per scheduler run
  }
}
