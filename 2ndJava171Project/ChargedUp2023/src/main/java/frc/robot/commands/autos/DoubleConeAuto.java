// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;


/** An example command that uses an example subsystem. */
public class DoubleConeAuto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DoubleConeAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem) {
    addCommands(
        new SuckInCone(rollersSubsystem),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition, ArmConstants.thirdLevelEncoderPosition),
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem), 
          new SpitOutCone(rollersSubsystem)),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe),
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem),
          new DriveForwardAuto(driveSubsystem, AutoConstants.secondConeDistance, false)),
        // gonna pick up a cube from the ground for safety
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.cubePickupEncoderPosition, ArmConstants.cubePickupEncoderPostion),
        new SuckInCone(rollersSubsystem),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe),
        new ParallelRaceGroup(
            new HoldPosition(wristSubsystem, armSubsystem),
            new DriveForwardAuto(driveSubsystem, AutoConstants.startingPosition, false)),
        new SuckInCone(rollersSubsystem),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.secondLevelEncoderPosition, ArmConstants.secondLevelEncoderPosition),
        new ParallelRaceGroup(
            new HoldPosition(wristSubsystem, armSubsystem),
            new SpitOutCone(rollersSubsystem)),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe),
        new ParallelRaceGroup(
            new HoldPosition(wristSubsystem, armSubsystem),
            new DriveForwardAuto(driveSubsystem, AutoConstants.distanceForward, false))
    );
    
  }
}