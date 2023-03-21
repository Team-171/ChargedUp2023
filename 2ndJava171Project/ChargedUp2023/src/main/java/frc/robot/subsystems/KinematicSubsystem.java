// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import frc.robot.RobotContainer;

import java.sql.Driver;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class KinematicSubsystem extends SubsystemBase {

  DifferentialDriveKinematics kinematics = 
    new DifferentialDriveKinematics(Units.inchesToMeters(23));

    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(7, 7);
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

    public AHRS ahrs = new AHRS();

    public boolean setup = false;

    public Pose2d m_pose;

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    // public DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
    //     ahrs.getRotation2d(),
    //     TankDriveSubsystem.leftLeadMotor.getEncoder().getPosition(), TankDriveSubsystem.rightLeadMotor.getEncoder().getPosition(),
    //     new Pose2d(5.0, 13.5, new Rotation2d()));

  /** Creates a new ExampleSubsystem. */
  public KinematicSubsystem() {
    
  }

  public void kinematics() {
    if(!setup){
        ahrs.calibrate();

        setup = true;
    }
    // RobotContainer.m_field.setRobotPose(m_pose);
  }

  @Override
  public void periodic() {
    // Get the rotation of the robot from the gyro.
    var gyroAngle = ahrs.getRotation2d();

    // Update the pose
    // m_pose = m_odometry.update(gyroAngle,
    //     TankDriveSubsystem.leftLeadMotor.getEncoder().getPosition(),
    //     TankDriveSubsystem.rightLeadMotor.getEncoder().getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
