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
  // Options for autos

  // Drive forward a set distance
  public static CommandBase driveForwardAuto(TankDriveSubsystem driveSubsystem) {
    return new DriveForwardAuto(driveSubsystem, AutoConstants.distanceForward, false);
  }

  // Places a cone on the top row and drives past the community line
  public static CommandBase simpleAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem){
    return new SimpleAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem);
  }

  // Places a cone on the top row and balances on the charge station
  public static CommandBase balanceAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem){
    return new BalanceAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem);
  }

  // Places a cone on the top row, drives over the charge station out of the community, back onto the charge station and balances
  public static CommandBase testBalanceAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem){
    return new TestBalanceAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem);
  }

  // Places a cube on the top row, drives out of the community and picks another up, and places that one on the second level
  public static CommandBase doubleCubeAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem){
    return new DoubleCubeAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
