// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;


/** An example command that uses an example subsystem. */
public class LeftDoublePieceWithCoop extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LeftDoublePieceWithCoop(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem) {
    addCommands(
        // Note: had to add distances together since it's a continuous measurement
        // suck in with less power and combine with set preset possibly to save time
        // can drive forward faster if need be for time
        new SuckInCone(rollersSubsystem),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition, ArmConstants.thirdLevelEncoderPosition),
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem), 
          new SpitOutCone(rollersSubsystem)),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe),
        new DriveForwardAuto(driveSubsystem, AutoConstants.clearFlushForTurnDistance, false),
        // check encoder distance change for left encoder
        new TurnAuto(driveSubsystem, -90, false),
        new DriveForwardAuto(driveSubsystem, AutoConstants.driveForwardFourFt + AutoConstants.clearFlushForTurnDistance, false),
        // check encoder distance again
        new TurnAuto(driveSubsystem, 90, false),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.cubePickupEncoderPosition, ArmConstants.cubePickupEncoderPostion),
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem),
          new DriveForwardAuto(driveSubsystem, AutoConstants.driveForwardLongDistance + AutoConstants.driveForwardFourFt + AutoConstants.clearFlushForTurnDistance, false),
          new SuckInCubeLong(rollersSubsystem)
        ),
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem),
          new DriveForwardAuto(driveSubsystem, AutoConstants.driveForwardLongDistance + AutoConstants.driveForwardFourFt + AutoConstants.clearFlushForTurnDistance - (AutoConstants.driveForwardLongDistance + AutoConstants.clearFlushForTurnDistance), false)
        ),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition, ArmConstants.thirdLevelEncoderPosition),
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem),
          new SpitOutCube(rollersSubsystem)
        ),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe),
        new HoldPosition(wristSubsystem, armSubsystem) 
    );
    
  }
}
