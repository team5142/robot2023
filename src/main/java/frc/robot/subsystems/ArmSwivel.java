// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSwivel extends SubsystemBase {
  /** Creates a new ArmSwivel. */

  // All Cone/Cube reference angles (For placement)
  private double highConeSwivelAngle = 0;

  private double lowConeSwivelAngle = 0;
  private double highCubeSwivelAngle = 0;
  private double lowCubeSwivelAngle = 0;
  private double groundHubSwivelAngle = 0;
  private double collectSwivelAngle = 0;
  private double substationSwivelAngle = 0;
  private double genericSwivelAngle = 0;

  // The TalonSRX motor controller that controls the swivel motion.
  private final TalonSRX swivelMotor;

  // The PID controller that adjusts the arm's position.
  private PIDController swivelPID;

  // The encoder that measures the arm's current position.
  private CANCoder encoder;

  // The proportional, integral, and derivative gains for the PID controller.
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // The acceptable tolerance in degrees for the PID controller.
  private static final double kToleranceDegrees = 2.0;

  // The top and bottom limit for the encoder's count in degrees.
  private static float kTopLimitEncoderDegrees = 0;
  private static float kBottomLimitEncoderDegrees = 180;

  public ArmSwivel() {
    swivelMotor = new TalonSRX(7);
    encoder = new CANCoder(10);
    encoder.configSensorDirection(false);

    swivelPID.setPID(kP, kI, kD);

    // Sets up the PID controller with the gains and tolerance.
    swivelPID = new PIDController(kP, kI, kD);
    swivelPID.setTolerance(kToleranceDegrees);

    // Sets the continuous input range for the PID controller
    swivelPID.enableContinuousInput(-1.0, 1.0);

    // clamps the output between -1.0 and 1.0.
    MathUtil.clamp(
        swivelPID.calculate(encoder.getAbsolutePosition(), genericSwivelAngle), -1.0, 1.0);
  }

  public void setHighConeSwivelAngle() {
    genericSwivelAngle = highConeSwivelAngle;
    setSwivelAngle(genericSwivelAngle);
  }

  public void setLowConeSwivelAngle() {
    genericSwivelAngle = lowConeSwivelAngle;
    setSwivelAngle(genericSwivelAngle);
  }

  public void setHighCubeSwivelAngle() {
    genericSwivelAngle = highCubeSwivelAngle;
    setSwivelAngle(genericSwivelAngle);
  }

  public void setLowCubeSwivelAngle() {
    genericSwivelAngle = lowCubeSwivelAngle;
    setSwivelAngle(genericSwivelAngle);
  }

  public void setGroundHubSwivelAngle() {
    genericSwivelAngle = groundHubSwivelAngle;
    setSwivelAngle(genericSwivelAngle);
  }

  public void setCollectSwivelAngle() {
    genericSwivelAngle = collectSwivelAngle;
    setSwivelAngle(genericSwivelAngle);
  }

  public void setSubstationSwivelAngle() {
    genericSwivelAngle = substationSwivelAngle;
    setSwivelAngle(genericSwivelAngle);
  }

  public void setSwivelAngle(double angleDegrees) {
    int targetEncoderCount = angleDegreesToEncoderCount(angleDegrees);

    if (targetEncoderCount < kTopLimitEncoderDegrees) {
      targetEncoderCount = (int) kTopLimitEncoderDegrees;
    } else if (targetEncoderCount > kBottomLimitEncoderDegrees) {
      targetEncoderCount = (int) kBottomLimitEncoderDegrees;
    }

    swivelPID.setSetpoint(targetEncoderCount);
    swivelPID.enableContinuousInput(angleDegrees, targetEncoderCount);
  }

  public void stopSwivel() {
    swivelPID.disableContinuousInput();
    swivelMotor.set(ControlMode.PercentOutput, 0.0);
  }

  private int angleDegreesToEncoderCount(double angleDegrees) {
    double encoderCountPerDegree = 10.0;
    return (int) (encoderCountPerDegree * angleDegrees);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
