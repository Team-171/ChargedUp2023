// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveConstants {
    public static final int kDriverControllerPort = 0;
    public static final int leftLeadDeviceID = 6;
    public static final int leftFollowDeviceID = 5;
    public static final int leftFollowDeviceID2= 4;
    public static final int rightLeadDeviceID = 9;
    public static final int rightFollowDeviceID = 8;
    public static final int rightFollowDeviceID2 = 7;

    public static final int driveMotorsCurrentLimit = 50;
  }

  public static class WristConstants {
    public static final int wristMotorID = 3;
    public static final double wristSpeed = 0.5;
    public static final String forwardButton = "RB";
    public static final String backwardButton = "LB";
  }

  public static class RollerConstants {
    public static final int rollerMotorID = 1;
  }
  
  public static class OperatorConstants {
    public static final int kOperatorControllerPort = 1;
    public static final int operatorLeftBumper = 5;
    public static final int operatorRightBumper = 6;
    public static final int rightTrigger = 2;
    public static final int leftTrigger = 3;
  }
  
  public static class ArmConstants {
    public static final int armMotorID = 2;
    public static final int armMotorID2 = 10;
  }

  public static class PneumaticConstants {
    public static final double minPressure = 40;
    public static final double maxPressure = 120;
    public static final int forwardChannel = 0;
    public static final int reverseChannel = 1;
    public static final int moduleID = 1; 
  }
}
