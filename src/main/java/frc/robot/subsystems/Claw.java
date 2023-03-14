// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  private final CANSparkMax m_clawMotor;
  private final DoubleSolenoid m_clawSolenoid;
  private Boolean m_isClosed = false; 
  public Claw() {
    m_clawMotor = new CANSparkMax(12, MotorType.kBrushless);
    m_clawSolenoid = new DoubleSolenoid(6, PneumaticsModuleType.CTREPCM, 0, 3);
    close();
  }

  public void setSpeed(double m_clawSpeed) {
    m_clawMotor.set(0.5);

  }
  private void close() {
    m_clawSolenoid.set(Value.kForward);
    m_isClosed = true;
  }

  private void open() {
    m_clawSolenoid.set(Value.kReverse);
    m_isClosed = false;
  }

  public void toggle() {
    if (m_isClosed == true) {
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
