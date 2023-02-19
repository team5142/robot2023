// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerPorts {
    public static final int kDriverControllerPort = 0;
    public static final int kForwardAxis = 0;
    public static final int kStrafeAxis = 3;
    public static final int kRotateAxis = 1;

    public static final int kOpertatorControllerPort = 1;
  }
      
      public static class DrivetrainConstants {
        public static final int wheelDiameter = 8; // inches
      }
    
      // follow CAN IDs in ascending order
      public static class CAN_IDs{
        //drive IDs
        public static final int frontLeftMotorID = 0;
        public static final int frontRightMotorID = 1;
        public static final int backLefttMotorID = 2;
        public static final int backRightMotorID = 3;
    
        //arm IDs
        public static final int SwivelID = 7;
        public static final int TelescopeID = 8;

        //Harvester ID
        public static final int harvesterID = 9;
    }
  }