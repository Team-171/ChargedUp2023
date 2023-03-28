// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeRollersSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.Constants.AutoConstants;

import edu.wpi.first.wpilibj2.command.CommandBase;

public final class Autos {

  /** Example static factory for an autonomous command. */
  public static CommandBase driveForwardAuto(TankDriveSubsystem driveSubsystem) {
    return new DriveForwardAuto(driveSubsystem, AutoConstants.distanceForward, false);
  }

  public static CommandBase simpleAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem){
    return new SimpleAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem);
  }

  public static CommandBase balanceAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem){
    return new BalanceAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem);
  }

  public static CommandBase testBalanceAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem){
    return new TestBalanceAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem);
  }

  public static CommandBase doubleConeAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem){
    return new DoubleConeAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
