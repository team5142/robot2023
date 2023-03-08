// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Import necessary WPILib and CTRE libraries
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
  // Creates a new Telescope

  // Declare class variables
  private final TalonSRX m_telescope; // TalonSRX motor controller for the telescope arm
  private final PIDController m_controller; // PID controller for position control

  // Declare constants for PID controller and distance range
  private final double kP = 0.1;
  private final double kI = 0.0;
  private final double kD = 0.0;

  public Telescope() {
    m_telescope = new TalonSRX(8);
    TalonSRXConfiguration configuration = new TalonSRXConfiguration();

    m_telescope.setNeutralMode(NeutralMode.Brake);
    m_telescope.configAllSettings(configuration);

    m_controller = new PIDController(0.1, 0, 0);
  }

  public void TelescopeOut() {
    m_telescope.set(ControlMode.PercentOutput, 0.35);
  }

  public void TelescopeIn() {
    m_telescope.set(ControlMode.PercentOutput, -0.2);
  }

  public void stop() {
    m_telescope.set(ControlMode.Position, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
