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
      // Suck in cone
        new SuckInCone(rollersSubsystem),
        // Sets the arm & wrist to third level position
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition, ArmConstants.thirdLevelEncoderPosition),
        // Hold the arm & wrist and spits out the cone
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem), 
          new SpitOutCone(rollersSubsystem)),
        // Goes into safe position for arm & wrist
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe),
        // Drives forward a set distance
        new DriveForwardAuto(driveSubsystem, AutoConstants.clearFlushForTurnDistance, false),
        // Turns 90 degrees
        new TurnAuto(driveSubsystem, -90, false),
        // Drives forward a set distance
        new DriveForwardAuto(driveSubsystem, AutoConstants.driveForwardFourFt + AutoConstants.clearFlushForTurnDistance, false),
        // Turns 90 degrees back the other way
        new TurnAuto(driveSubsystem, 90, false),
        // Sets the arm & wrist to cone pickup position
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.cubePickupEncoderPosition, ArmConstants.cubePickupEncoderPostion),
        // Holds the position of wrist & arm, drives forward a set distance, and sucks in the cube
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem),
          new DriveForwardAuto(driveSubsystem, AutoConstants.driveForwardLongDistance + AutoConstants.driveForwardFourFt + AutoConstants.clearFlushForTurnDistance, false),
          new SuckInCubeLong(rollersSubsystem)
        ),
        // Holds the position and drives forward a set distance
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem),
          new DriveForwardAuto(driveSubsystem, AutoConstants.driveForwardLongDistance + AutoConstants.driveForwardFourFt + AutoConstants.clearFlushForTurnDistance - (AutoConstants.driveForwardLongDistance + AutoConstants.clearFlushForTurnDistance), false)
        ),
        // Set the arm & wrist to the third level position
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition, ArmConstants.thirdLevelEncoderPosition),
        // Holds arm and wrist position and spits out the cube
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem),
          new SpitOutCube(rollersSubsystem)
        ),
        // Goes back into safe position
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe),
        // Holds the arm & wrist at current position
        new HoldPosition(wristSubsystem, armSubsystem) 
    );
    
  }
}
