// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;


/** An example command that uses an example subsystem. */
public class DoubleCubeAuto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DoubleCubeAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem) {
    addCommands(
      // suck in cone
        new SuckInCone(rollersSubsystem),
        // go to shoot position third level
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition, ArmConstants.thirdLevelEncoderPosition),
        // shoot
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem), 
          new SpitOutCone(rollersSubsystem)),
        // go to cube pickup position
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.cubePickupEncoderPosition, ArmConstants.cubePickupEncoderPostion),
        // drive forward and pick up cube
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem),
          new SuckInCubeLong(rollersSubsystem),
          new DriveForwardAuto(driveSubsystem, AutoConstants.secondCubeDistance, false)),
        // drive back to start
        new ParallelRaceGroup(
            new HoldPosition(wristSubsystem, armSubsystem),
            new DriveForwardAuto(driveSubsystem, AutoConstants.startingPosition, false)),
        // go to second level preset
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.secondLevelEncoderPosition, ArmConstants.secondLevelEncoderPosition),
        // spit cube out on second level
        new ParallelRaceGroup(
            new HoldPosition(wristSubsystem, armSubsystem),
            new SpitOutCube(rollersSubsystem)),
        // go to safe position
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe)
        // drive across the line
        // new ParallelRaceGroup(
        //     new HoldPosition(wristSubsystem, armSubsystem),
        //     new DriveForwardAuto(driveSubsystem, AutoConstants.distanceForward, false))
    );
    
  }
}
