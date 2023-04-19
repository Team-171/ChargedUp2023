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
    public static final double balanceDeadZone = 3.0;

    public static final int selectControllerbutton = 7;
    public static final int startControllerButton = 8;

    public static final double defaultSpeed = 1.0;
    public static final double slowSpeed = 0.3;
    public static final double slowForward = 0.5;

    public static final int rightTrigger = 2;
    public static final int leftTrigger = 3;
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
    public static final double wristRoughMiddle = 0.24;
    public static final double wristLowHardStop = -0.7;
    public static final double wristHighHardStop = 0.2;
    public static final double wristSpeed = 0.75;
    public static final double wristReturnSpeed = 0.25;
    public static final double conePickupEncoderPosition = 0.16;
    public static final double thirdLevelEncoderPosition = -0.55;
    public static final double cubePickupEncoderPosition = 0.07;
    public static final double secondLevelEncoderPosition = -0.45;
    public static final double inputConeEncoderPosition = -0.42;
    public static final double inputCubeEncoderPosition = -0.55;
    public static final double standingConePosition = -0.07;
    public static final double reset = 0.07; 
    public static final double safe = -0;
    public static final double absoluteRoughMiddle = 0.5;
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
    public static final double armPIDkp = 7;
    public static final double armPIDki = 0.1;
    public static final double armPIDkd = 0.1;
    public static final double armDeadZone = 0.1;
    public static final double armRoughMiddle = 0.32;
    public static final double armLowHardStop = 0.19;
    public static final double armHighHardStop = 0.61;
    public static final double armSpeed = 0.5;
    public static final double armReturnSpeed = 0.25;
    public static final double conePickupEncoderPosition = 0.21;
    public static final double thirdLevelEncoderPosition = 0.58;
    public static final double cubePickupEncoderPostion = 0.195;
    public static final double secondLevelEncoderPosition = 0.57;
    public static final double inputConeEncoderPosition = 0.55;
    public static final double inputCubeEncoderPosition = 0.59;
    public static final double standingConePosition = 0.32;
    public static final double reset = 0.36;
    public static final double safe = 0.375;
  }

  public static class PneumaticConstants {
    public static final double minPressure = 40;
    public static final double maxPressure = 90;
    public static final int forwardChannel = 0;
    public static final int reverseChannel = 1;
    public static final int moduleID = 1; 
  }

  public static class AutoConstants {
    //Multiply wanted inches by conversion factor (0.7854166666666673)

    public static final double startingPosition = 0;
    public static final double distanceForward = 172; //
    public static final double balanceDistanceForward = 115; //90
    public static final double slowBalanceDistance = 83; //65
    public static final double crossLineDistance = 170; //145
    public static final double backDistance = 78; //60.5
    public static final double secondCubeDistance = 180; //195
    public static final double armTolerance = 0.02;
    public static final double wristTolerance = 0.02;
    public static final double driveTolerance = 5.0;
    public static final double driveForwardSpeed = 0.75;
    public static final double driveBackwardSpeed = 0.5;
    public static final double driveSlowlyMultiplier = 0.75;
    public static final double intakeSpeedCubePickup = 0.3;
    public static final double grabConeDistance = 9;
    public static final double clearFlushForTurnDistance = 7.64; // 6 inches
    public static final double turnToleranceForYaw = 1.0;
    public static final double driveForwardFourFt = 61.11; // 4 ft
    public static final double driveForwardLongDistance = 236.82; //186 inches
  }
}
