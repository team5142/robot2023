// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class ledStrip extends SubsystemBase {
  /** Creates a new ledStrip. */
  private final int kLength = 39; // Length of the LED strip
  private AddressableLED m_Led;
  private AddressableLEDBuffer m_LedBuffer;

  public void LedStrip() {
      m_Led = new AddressableLED(0);
      m_LedBuffer = new AddressableLEDBuffer(kLength);
      m_Led.setLength(kLength);
      m_Led.setData(m_LedBuffer);
      m_Led.start();
  }
  public void setColor(Color color) {
    for (int i = 0; i < kLength; i++) {
        m_LedBuffer.setLED(i, color);
    }
    m_Led.setData(m_LedBuffer);
}

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Led.start();
  }
}
