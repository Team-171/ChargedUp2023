// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;

/**
 * Creates a balance auto
 */
public class BalanceAuto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  public BalanceAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem) {
    addCommands(
      // Suck in cone
        new SuckInCone(rollersSubsystem),
      // Sets arm & wrist to third level position
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition, ArmConstants.thirdLevelEncoderPosition),
      // Holds the arm & wrist and spits out the cone
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem), 
          new SpitOutCone(rollersSubsystem)),
      // Goes into reset position for arm & wrist
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.reset, ArmConstants.reset),
      // Holds arm & wrist position and drives forward a set distance
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem),
          new DriveForwardAuto(driveSubsystem, AutoConstants.balanceDistanceForward, false)),
      // Holds arm & wrist position and balances on the charge station
          new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem),
          new BalanceCommand(driveSubsystem)
        )
    );
    
  }
}
