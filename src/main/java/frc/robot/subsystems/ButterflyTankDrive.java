// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.playingwithfusion.CANVenom;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;

public class ButterflyTankDrive extends SubsystemBase {
//Creates a single solenoid that controlls all four butterfly modules, dropping/retracting the traction wheels
DoubleSolenoid ButterflySolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

//All motor controllers for the VENOM motors used in the drivetrain
private final MotorController m_frontLeftMotor = new CANVenom(0);
private final MotorController m_frontRightMotor = new CANVenom(1);
private final MotorController m_backLeftMotor = new CANVenom(2);
private final MotorController m_backRightMotor = new CANVenom(3);

//All motor encoders for the VENOM motors used in the drivetrain
private final Encoder m_frontLeftEncoder = new Encoder(0, 1);
private final Encoder m_frontRightEncoder = new Encoder(2, 3);
private final Encoder m_backLeftEncoder = new Encoder(4, 5);
private final Encoder m_backRightEncoder = new Encoder(6, 7);

  /** Creates a new ButterflyDrive. */
  public ButterflyTankDrive() {
  
// Gets the state of the solenoid (True/False)

/* WILL FINISH LATER
public boolean ButterflyActuationState(){
  return ButterflySolenoid.get();}
*/
  
  

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}