// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  // Define the elevator motor and encoder
  private CANVenom m_elevator;
  private DigitalInput m_limit;

  // Constructor for the elevator subsystem
  public Elevator() {
    m_elevator = new CANVenom(4);
    m_elevator.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_elevator.setKP(0.35);
    m_elevator.setControlMode(ControlMode.PositionControl);
    m_limit = new DigitalInput(1);
    // m_elevator.resetPosition();
    // toBottom();
  }

  // Method to stop the elevator
  public void stop() {
    m_elevator.set(0);
  }

  public void toBottom() {
    m_elevator.setCommand(ControlMode.PositionControl, 0);
  }

  public void toLower() {
    m_elevator.setPosition(250);
  }

  public void toMid() {
    m_elevator.setPosition(500);
  }

  public void toTop() {
    m_elevator.setPosition(1000);
  }

  public void setSpeed(double speed){
    m_elevator.set(speed);
  }

  //  public void elevatorMovement(double upSpeed, double downSpeed) {
  //    m_elevator.arcadeDrive(upSpeed, downSpeed);
  //  }

  public void elevatorUpPower(double ePower) {
    m_elevator.set(ePower); // Sets a constant holding power (To resist gravity)
  }

  public double getEncoder() {
    return m_elevator.getPosition();
  }

  @Override
  public void periodic() {
    // if (m_limit.get() == true) {
    //   m_elevator.setPosition(0);
    //   m_elevator.set(0);
    // }
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ElevatorEnc", getEncoder());
    SmartDashboard.putBoolean("ElevatorLimt", m_limit.get());
  }

  public static void set(double speed) {
  }
}
