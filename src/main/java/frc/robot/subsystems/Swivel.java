// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swivel extends SubsystemBase {
  /** Creates a new Swivel. */

  // All Cone/Cube reference angles (For placement)
  private double collectSwivelAngle = 0;

  private double lowSwivelAngle = 0;
  public double midSwivelAngle = 130;
  private double highSwivelAngle = 0;

  // The TalonSRX motor controller that controls the swivel motion.
  private final TalonSRX m_swivel;
  private final CANCoder m_encoder;
  private final PIDController m_controller;

  public Swivel() {
    m_swivel = new TalonSRX(8);
    m_encoder = new CANCoder(11);

    m_swivel.setNeutralMode(NeutralMode.Brake);
    m_encoder.configFactoryDefault();
    m_encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    m_controller = new PIDController(0.2, 0, 0.04);
  }

  public void swivelUp() {
    m_swivel.set(ControlMode.PercentOutput, 0.35, DemandType.ArbitraryFeedForward, 0.15);
  }

  public void swivelDown() {
    m_swivel.set(ControlMode.PercentOutput, -0.2);
  }

  public void stop() {
    m_swivel.set(ControlMode.PercentOutput, 0);
  }

  public void swivelLow() {
    m_swivel.set(ControlMode.Position, lowSwivelAngle);
  }

  public void swivelMid() {
    m_swivel.set(ControlMode.PercentOutput, -m_controller.calculate(m_encoder.getAbsolutePosition(), midSwivelAngle));
  }

  public void swivelHigh() {
    m_swivel.set(ControlMode.Position, highSwivelAngle);
  }

  public void swivelCollect() {
    m_swivel.set(ControlMode.Position, collectSwivelAngle);
  }

  public double getEncoder() {
    return m_encoder.getAbsolutePosition();
  }

  public void initialize() {
    m_swivel.set(ControlMode.Position, m_controller.calculate(m_encoder.getPosition(), 350));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SwivelEnc", getEncoder());
    // This method will be called once per scheduler run
  }
}
