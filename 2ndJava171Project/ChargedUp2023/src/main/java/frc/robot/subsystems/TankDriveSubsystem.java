// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class TankDriveSubsystem extends SubsystemBase {

  CANSparkMax leftLeadMotor;
  CANSparkMax leftFollowMotor;
  CANSparkMax leftFollowMotor2;
  CANSparkMax rightLeadMotor;
  CANSparkMax rightFollowMotor;
  CANSparkMax rightFollowMotor2;

  DifferentialDrive roboDrive;

  // unused right now
  private DifferentialDriveKinematics kinematics = 
    new DifferentialDriveKinematics(Units.inchesToMeters(23));
  private DifferentialDriveWheelSpeeds wheelSpeeds = 
    new DifferentialDriveWheelSpeeds(1.6, 1.6);

  private AHRS ahrs;

  public static boolean setup = false;

  private static Pose2d m_pose;
  private DifferentialDriveOdometry m_odometry;
  private Field2d m_field;

  private PIDController drivePid;
  private PIDController balancePid;

  public double turnMultiplier;
  public double forwardMultiplier;

  public double currentHeading;

  /** Creates a new ExampleSubsystem. */
  public TankDriveSubsystem() {
    leftLeadMotor = new CANSparkMax(DriveConstants.leftLeadDeviceID, MotorType.kBrushless);
    leftFollowMotor = new CANSparkMax(DriveConstants.leftFollowDeviceID, MotorType.kBrushless);
    leftFollowMotor2 = new CANSparkMax(DriveConstants.leftFollowDeviceID2, MotorType.kBrushless);
    rightLeadMotor = new CANSparkMax(DriveConstants.rightLeadDeviceID, MotorType.kBrushless);
    rightFollowMotor = new CANSparkMax(DriveConstants.rightFollowDeviceID, MotorType.kBrushless);
    rightFollowMotor2 = new CANSparkMax(DriveConstants.rightFollowDeviceID2, MotorType.kBrushless);

    leftLeadMotor.restoreFactoryDefaults();
    leftFollowMotor.restoreFactoryDefaults();
    leftFollowMotor2.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();
    rightFollowMotor.restoreFactoryDefaults();
    rightFollowMotor2.restoreFactoryDefaults();

    leftLeadMotor.setInverted(true);
    leftFollowMotor.setInverted(true);
    leftFollowMotor2.setInverted(true);

    leftLeadMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    leftFollowMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    leftFollowMotor2.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    rightLeadMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    rightFollowMotor.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);
    rightFollowMotor2.setSmartCurrentLimit(DriveConstants.driveMotorsCurrentLimit);

    leftLeadMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    leftFollowMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    leftFollowMotor2.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    rightLeadMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    rightFollowMotor.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);
    rightFollowMotor2.setClosedLoopRampRate(DriveConstants.driveMotorsRampRate);

    leftFollowMotor.follow(leftLeadMotor);
    leftFollowMotor2.follow(leftLeadMotor);
    rightFollowMotor.follow(rightLeadMotor);
    rightFollowMotor2.follow(rightLeadMotor);

    roboDrive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);

    leftLeadMotor.getEncoder().setPosition(0);
    rightLeadMotor.getEncoder().setPosition(0);

    leftLeadMotor.getEncoder().setPositionConversionFactor(0.7854166666666673);
    rightLeadMotor.getEncoder().setPositionConversionFactor(0.7854166666666673);
    //One rotation is 0.78 inches

    ahrs = new AHRS();

    m_odometry = new DifferentialDriveOdometry(
        ahrs.getRotation2d(),
        leftLeadMotor.getEncoder().getPosition(), rightLeadMotor.getEncoder().getPosition(),
        new Pose2d(2, 1, new Rotation2d())); 

    m_field = new Field2d();
    m_pose = new Pose2d();

    currentHeading = ahrs.getYaw();

    turnMultiplier = DriveConstants.defaultSpeed;
    forwardMultiplier = DriveConstants.defaultSpeed;


    SmartDashboard.putData("Field: ", m_field);
    
    drivePid = new PIDController(3, 0.5, 0.5);
    // not used right now
    balancePid = new PIDController(.6, 3, 7);
  }

  public void arcadeDrive(double forward, double rotation, Boolean slowTurn){
    if(slowTurn){
      turnMultiplier = DriveConstants.slowSpeed;
      forwardMultiplier = DriveConstants.slowForward;
    }

    roboDrive.arcadeDrive(forward * forwardMultiplier, rotation * turnMultiplier);

    turnMultiplier = DriveConstants.defaultSpeed;
    forwardMultiplier = DriveConstants.defaultSpeed;
  }

  public boolean driveForward(double length, boolean slowMode){
    if(slowMode){
      forwardMultiplier = AutoConstants.driveSlowlyMultiplier;
    }

    SmartDashboard.putNumber("Left Encoder:" , leftLeadMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Length: ", length);
    
    roboDrive.arcadeDrive(MathUtil.clamp(drivePid.calculate(leftLeadMotor.getEncoder().getPosition(), length), -0.75, 0.75) * forwardMultiplier, 0);

    forwardMultiplier = DriveConstants.defaultSpeed;

    if(leftLeadMotor.getEncoder().getPosition() > length - AutoConstants.driveTolerance && leftLeadMotor.getEncoder().getPosition() < length + AutoConstants.driveTolerance){
      return true;
    }

    return false;
  }

  public boolean turn(double turnDegrees, boolean slowMode){
    if(slowMode){
      turnMultiplier = DriveConstants.slowForward;
    }

    double newHeading = currentHeading + turnDegrees;

    roboDrive.arcadeDrive(0, MathUtil.clamp(drivePid.calculate(ahrs.getYaw(), newHeading), -.75, .75) * turnMultiplier);

    turnMultiplier = DriveConstants.defaultSpeed;

    if(ahrs.getYaw() > newHeading - AutoConstants.turnToleranceForYaw && ahrs.getYaw() < newHeading + AutoConstants.turnToleranceForYaw){
      return true;
    }
    
    return false;
    
  }

  public void resetYaw(){
    currentHeading = ahrs.getYaw();
  }

  public void setup(){
    ahrs.calibrate();
  }

  public void balance(){
    // if(ahrs.getRoll() > DriveConstants.balanceDeadZone || ahrs.getRoll() < -DriveConstants.balanceDeadZone){
    //   roboDrive.arcadeDrive(-MathUtil.clamp(balancePid.calculate(ahrs.getRoll(), 0), -0.4, 0.4), 0);
    // }else{
    //   roboDrive.arcadeDrive(0, 0);
    //   Timer.delay(1);
    // }

    if(ahrs.getRoll() > DriveConstants.balanceDeadZone){
      roboDrive.arcadeDrive(0.25, 0);
    }else if(ahrs.getRoll() < -DriveConstants.balanceDeadZone){
      roboDrive.arcadeDrive(-0.25, 0);
    }else{
      roboDrive.arcadeDrive(0, 0);
    }
  }

  @Override
  public void periodic() {
   
    SmartDashboard.putNumber("Left Drive Speed: ", leftLeadMotor.get());
    SmartDashboard.putNumber("Right Drive Speed: ", rightLeadMotor.get());
    SmartDashboard.putNumber("Left Encoder:" , leftLeadMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("Gyro Pitch: ", ahrs.getPitch());
    SmartDashboard.putNumber("Gyro Roll: ", ahrs.getRoll());
    SmartDashboard.putNumber("Gyro Yaw: ", ahrs.getYaw());

    var gyroAngle = ahrs.getRotation2d();

    // Update the pose
    m_pose = m_odometry.update(gyroAngle,
        leftLeadMotor.getEncoder().getPosition(),
        rightLeadMotor.getEncoder().getPosition());
    
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
