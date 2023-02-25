// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ButterflyPneumatics extends SubsystemBase {
  /** Creates a new Butterfly. */
  
  DoubleSolenoid ButterflySolenoid;

  public ButterflyPneumatics() {
    // Initialize the double solenoid object with the appropriate ports
    ButterflySolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  }

  public void extend() {
    // Extend the double solenoid
    ButterflySolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    // Retract the double solenoid
    ButterflySolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void stop() {
    // Stop the double solenoid
    ButterflySolenoid.set(DoubleSolenoid.Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}