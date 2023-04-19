// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class TankDriveSubsystem extends SubsystemBase {
  // Initialization of variables
  CANSparkMax leftLeadMotor;
  CANSparkMax leftFollowMotor;
  CANSparkMax leftFollowMotor2;
  CANSparkMax rightLeadMotor;
  CANSparkMax rightFollowMotor;
  CANSparkMax rightFollowMotor2;

  DifferentialDrive roboDrive;

  private AHRS gyro;

  public static boolean setup = false;

  private PIDController drivePid;

  public double turnMultiplier;
  public double forwardMultiplier;

  public double currentHeading;

  /** 
   * Creates a new TankDriveSubsystem.
   * Controls the base driving 
  */
  public TankDriveSubsystem() {
    // Creates all the motors
    leftLeadMotor = new CANSparkMax(DriveConstants.leftLeadDeviceID, MotorType.kBrushless);
    leftFollowMotor = new CANSparkMax(DriveConstants.leftFollowDeviceID, MotorType.kBrushless);
    leftFollowMotor2 = new CANSparkMax(DriveConstants.leftFollowDeviceID2, MotorType.kBrushless);
    rightLeadMotor = new CANSparkMax(DriveConstants.rightLeadDeviceID, MotorType.kBrushless);
    rightFollowMotor = new CANSparkMax(DriveConstants.rightFollowDeviceID, MotorType.kBrushless);
    rightFollowMotor2 = new CANSparkMax(DriveConstants.rightFollowDeviceID2, MotorType.kBrushless);

    // Resets to default, always do before changing config
    leftLeadMotor.restoreFactoryDefaults();
    leftFollowMotor.restoreFactoryDefaults();
    leftFollowMotor2.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();
    rightFollowMotor.restoreFactoryDefaults();
    rightFollowMotor2.restoreFactoryDefaults();

    // Sets some motors to inverted so they work together
    leftLeadMotor.setInverted(true);
    leftFollowMotor.setInverted(true);
    leftFollowMotor2.setInverted(true);

    // Sets current limit to not fry motors
    leftLeadMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    leftFollowMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    leftFollowMotor2.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    rightLeadMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    rightFollowMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    rightFollowMotor2.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);

    // Ramp rates to prevent brownouts
    leftLeadMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    leftFollowMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    leftFollowMotor2.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    rightLeadMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    rightFollowMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    rightFollowMotor2.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);

    // Motors follow lead motor with speed
    leftFollowMotor.follow(leftLeadMotor);
    leftFollowMotor2.follow(leftLeadMotor);
    rightFollowMotor.follow(rightLeadMotor);
    rightFollowMotor2.follow(rightLeadMotor);

    // Creates the differential drive
    roboDrive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);

    // Resets built in motors
    leftLeadMotor.getEncoder().setPosition(0);
    rightLeadMotor.getEncoder().setPosition(0);

    // Conversion factor to know how far it traveled, mainly used in autonomous
    leftLeadMotor.getEncoder().setPositionConversionFactor(0.7854166666666673);
    rightLeadMotor.getEncoder().setPositionConversionFactor(0.7854166666666673);
    //One rotation is 0.78 inches

    // Creates gyro
    gyro = new AHRS();

    // Gets current direction the robot is pointing
    currentHeading = gyro.getYaw();

    // Sets default speed
    turnMultiplier = DriveConstants.defaultSpeed;
    forwardMultiplier = DriveConstants.defaultSpeed;
    
    // Sets up pid for drive
    drivePid = new PIDController(3, 0.5, 0.5);
  }

  /**
   * Drives the robot
   * @param forward double Speed forward from controller input
   * @param rotation double Rotation speed from controller input
   * @param slowTurn boolean Slows down speed from a button press
   */
  public void arcadeDrive(double forward, double rotation, Boolean slowTurn){
    // Sets speed slower if the slow speed button is pressed
    if(slowTurn){
      turnMultiplier = DriveConstants.slowSpeed;
      forwardMultiplier = DriveConstants.slowForward;
    }

    // Drives the robot forward, and/or rotates it
    roboDrive.arcadeDrive(forward * forwardMultiplier, rotation * turnMultiplier);

    // Sets the speed back to default
    turnMultiplier = DriveConstants.defaultSpeed;
    forwardMultiplier = DriveConstants.defaultSpeed;
  }

  /**
   * Drives the robot forward a set distance
   * Mainly used in autonomous
   * @param length double Distance to drive forward
   * @param slowMode boolean If robot should slow down in driving
   * @return boolean If the robot reached its destination 
   */
  public boolean driveForward(double length, boolean slowMode){
    // Slows down forward speed if slowMode is true
    if(slowMode){
      forwardMultiplier = AutoConstants.driveSlowlyMultiplier;
    }

    // SmartDashboard posting
    SmartDashboard.putNumber("Left Encoder:" , leftLeadMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Length: ", length);
    
    // Drives the robot forward until it reaches the destination
    roboDrive.arcadeDrive(MathUtil.clamp(drivePid.calculate(leftLeadMotor.getEncoder().getPosition(), length), -0.75, 0.75) * forwardMultiplier, 0);

    // Sets the speed back to default
    forwardMultiplier = DriveConstants.defaultSpeed;

    // Returns true when the robot reaches the destination
    if(leftLeadMotor.getEncoder().getPosition() > length - AutoConstants.driveTolerance && leftLeadMotor.getEncoder().getPosition() < length + AutoConstants.driveTolerance){
      return true;
    }

    return false;
  }

  /**
   * Turns the robot
   * Mainly used in autonomous
   * @param turnDegrees double Degrees to turn the robot
   * @param slowMode boolean If the robot should turn slowly
   * @return boolean If the robot is finished turning
   */
  public boolean turn(double turnDegrees, boolean slowMode){
    // Slows down turn speed if slowMode is true
    if(slowMode){
      turnMultiplier = DriveConstants.slowForward;
    }

    // Sets where the destination rotation should be
    double newHeading = currentHeading + turnDegrees;

    // Rotates the robot until reaching the correct rotation
    roboDrive.arcadeDrive(0, MathUtil.clamp(drivePid.calculate(gyro.getYaw(), newHeading), -.75, .75) * turnMultiplier);

    // Sets the speed back to default
    turnMultiplier = DriveConstants.defaultSpeed;

    // Returns true when the robot's rotation is the desired rotation
    if(gyro.getYaw() > newHeading - AutoConstants.turnToleranceForYaw && gyro.getYaw() < newHeading + AutoConstants.turnToleranceForYaw){
      return true;
    }
    
    return false;
    
  }

  /**
   * Sets where the robot should be pointing to where it is pointing
   */
  public void resetYaw(){
    currentHeading = gyro.getYaw();
  }

  /**
   * Calibrates the gyro
   * Don't move while calibrating
   */
  public void setup(){
    gyro.calibrate();
  }

  /**
   * Auto balances the robot by driving forward and backward
   */
  public void balance(){

    if(gyro.getRoll() > DriveConstants.balanceDeadZone){
      // Drives forward to balance the robot
      roboDrive.arcadeDrive(0.25, 0);
    }else if(gyro.getRoll() < -DriveConstants.balanceDeadZone){
      // Drives backward to balance the robot
      roboDrive.arcadeDrive(-0.25, 0);
    }else{
      // Stop driving if it is balanced
      roboDrive.arcadeDrive(0, 0);
    }
  }

  @Override
  public void periodic() {
    // SmartDashboard posting
    // Put SmartDashboard in periodic so the output is always working even when the robot is disabled
   
    SmartDashboard.putNumber("Left Drive Speed: ", leftLeadMotor.get());
    SmartDashboard.putNumber("Right Drive Speed: ", rightLeadMotor.get());
    SmartDashboard.putNumber("Left Encoder:" , leftLeadMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("Gyro Pitch: ", gyro.getPitch());
    SmartDashboard.putNumber("Gyro Roll: ", gyro.getRoll());
    SmartDashboard.putNumber("Gyro Yaw: ", gyro.getYaw());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
