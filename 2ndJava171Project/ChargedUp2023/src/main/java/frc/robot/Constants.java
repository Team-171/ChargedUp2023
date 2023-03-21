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
    public static final double driveMotorsRampRate = 0.125;
  }

  public static class WristConstants {
    public static final int wristMotorID = 3;
    public static final int wristEncoderChannel = 1;
    public static final String forwardButton = "RB";
    public static final String backwardButton = "LB";
    public static final double wristPIDkp = 2;
    public static final double wristPIDki = 0.05;
    public static final double wristPIDkd = 0.1;
    public static final double wristDeadZone = 0.1;
    public static final double wristRoughMiddle = 0.2;
    public static final double wristLowHardStop = -1.5;
    public static final double wristHighHardStop = 1.45;
    public static final double wristSpeed = 0.75;
    public static final double wristReturnSpeed = 0.25;
    public static final double aButton = 0.656;
    public static final double bButton = -0.9;
    public static final double xButton = 0.464;
    public static final double yButton = -0.44;
    public static final double reset = 0.471; 
  }

  public static class RollerConstants {
    public static final int rollerMotorID = 1;
    public static final double rollerPIDkp = 0.025;
    public static final double rollerPIDki = 0;
    public static final double rollerPIDkd = 0;
    public static final double rollerDeadZone = 0.1;
    public static final double rollerSpeed = 1;
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
    public static final int armEncoderChannel = 0;
    public static final double armPIDkp = 8;
    public static final double armPIDki = 0.1;
    public static final double armPIDkd = 0.1;
    public static final double armDeadZone = 0.1;
    public static final double armRoughMiddle = 0.16;
    public static final double armLowHardStop = 0.03;
    public static final double armHighHardStop = 0.45;
    public static final double armSpeed = 1;
    public static final double armReturnSpeed = 0.25;
    public static final double aButton = 0.05;
    public static final double bButton = 0.4;
    public static final double xButton = 0.057;
    public static final double yButton = 0.38;
    public static final double reset = 0.2; 
  }

  public static class PneumaticConstants {
    public static final double minPressure = 40;
    public static final double maxPressure = 120;
    public static final int forwardChannel = 0;
    public static final int reverseChannel = 1;
    public static final int moduleID = 1; 
  }
}
