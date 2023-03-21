// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.PIDConstants;
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

  public DifferentialDriveKinematics kinematics = 
    new DifferentialDriveKinematics(Units.inchesToMeters(23));

    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(1.6, 1.6);
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

    public AHRS ahrs = new AHRS();

    public static boolean setup = false;

    public static Pose2d m_pose;

    public DifferentialDriveOdometry m_odometry;

    public Field2d m_field;

    public PIDController pid;

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
    leftLeadMotor.getEncoder().setPositionConversionFactor(0.0416666666666667);
    rightLeadMotor.getEncoder().setPositionConversionFactor(0.0416666666666667);

    m_odometry = new DifferentialDriveOdometry(
        ahrs.getRotation2d(),
        leftLeadMotor.getEncoder().getPosition(), rightLeadMotor.getEncoder().getPosition(),
        new Pose2d(2, 1, new Rotation2d())); 

    m_field = new Field2d();

    SmartDashboard.putData("Field: ", m_field);
    
    pid = new PIDController(3, 0.5, 0.5);
  }

  public void arcadeDrive(double forward, double rotation){
    roboDrive.arcadeDrive(forward, rotation);
  }

  public void driveForward(double length){

    SmartDashboard.putNumber("Left Encoder:" , leftLeadMotor.getEncoder().getPosition());
    roboDrive.arcadeDrive(MathUtil.clamp(pid.calculate(leftLeadMotor.getEncoder().getPosition(), length), -0.5, 0.5), 0);
  }

  public void setup(){
    ahrs.calibrate();
  }

  @Override
  public void periodic() {
   
    SmartDashboard.putNumber("Left Drive Speed: ", leftLeadMotor.get());
    SmartDashboard.putNumber("Right Drive Speed: ", rightLeadMotor.get());

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
