// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private final TalonSRX m_clawMotor;
  private final DoubleSolenoid m_clawSolenoid;
  private Boolean m_isFlipped = false;

  public Claw() {
    m_clawMotor = new TalonSRX(12);
    m_clawSolenoid = new DoubleSolenoid(6, PneumaticsModuleType.CTREPCM, 0, 3);
    close();
  }

  public void intake() {
    m_clawMotor.set(ControlMode.PercentOutput, 0.5); // Sets motor output to +50%
  }

  public void outtake() {
    m_clawMotor.set(ControlMode.PercentOutput, -0.5); // Sets motor output to -50%
  }

  private void open() {
    m_clawSolenoid.set(Value.kReverse);
    m_isFlipped = false;
  }

  private void close() {
    m_clawSolenoid.set(Value.kForward);
    m_isFlipped = true;
  }

  public void toggle() {
    if (m_isFlipped == true) {
      open();
    } else {
      close();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
