// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.playingwithfusion.CANVenom.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  // Define constants for the elevator
  private static final double kP = 0.1;

  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kF = 0.0;
  private static final double kTolerance = 0.5;

  private static final int kElevatorEncoderCountsPerInch = 4096;
  private static final int kElevatorMaxEncoderCounts = 10000;
  private static final int kElevatorMinEncoderCounts = 0;

  // Define the elevator motor and encoder
  private CANVenom m_elevator;

  // Define the PID controller for the elevator
  private PIDController elevatorPID;

  // Constructor for the elevator subsystem
  public Elevator() {
    m_elevator = new CANVenom(4);
    m_elevator.setBrakeCoastMode(BrakeCoastMode.Brake);
    m_elevator.setKP(0.35);
    m_elevator.setControlMode(ControlMode.Proportional);
    m_elevator.resetPosition();
  }

  // Method to stop the elevator
  public void stop() {
    m_elevator.set(0);
  }

  public void toBottom() {
    m_elevator.setPosition(0);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
