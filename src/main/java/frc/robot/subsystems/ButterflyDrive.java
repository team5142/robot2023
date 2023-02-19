// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;

public class ButterflyDrive extends SubsystemBase {
  /** Creates a new Butterfly. */
  public ButterflyDrive() {}
//Creates an xbox controller at port 0 that controls the solenoid in this subsystem
  private final XboxController m_controller = new XboxController(0);
//Creates a single solenoid that controlls all four butterfly modules, dropping/retracting the traction wheels
  Solenoid ButterflySolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
//If the Y button was pressed on the controller, the solenoid on/off
    if (m_controller.getYButtonPressed()) {
       ButterflySolenoid.toggle();
    }
  }
}