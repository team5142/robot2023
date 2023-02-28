// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Encoder;

public class ArmSwivel extends SubsystemBase {
  /** Creates a new ArmSwivel. */

  //All Cone/Cube reference angles (For placement)
  private double highConeSwivelAngle = 0;
  private double lowConeSwivelAngle= 0;
  private double highCubeSwivelAngle = 0;
  private double lowCubeSwivelAngle = 0;
  private double groundHubSwivelAngle = 0;
  private double collectSwivelAngle = 0;
  private double substationSwivelAngle = 0;

  private final TalonSRX swivelMotor;

  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private PIDController swivelPID = new PIDController(kP, kI, kD);

  public ArmSwivel() {
    swivelMotor = new TalonSRX(7);
    
    swivelPID.setPID(kP, kI, kD);
    swivelPID.enableContinuousInput(-1.0, 1.0);
    //MathUtil.clamp(swivelPID.calculate(encoder.getDistance(), setpoint), -1.0, 1.0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  }
