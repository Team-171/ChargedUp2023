// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;


/** A 9 point auto */
public class SimpleAuto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new SimpleAuto command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SimpleAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem) {
    addCommands(
        // suck in the cone
        new SuckInCone(rollersSubsystem),
        // got to the third level scoring position
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition, ArmConstants.thirdLevelEncoderPosition),
        new ParallelRaceGroup(
          // hold the current position
          new HoldPosition(wristSubsystem, armSubsystem), 
          // score the cone
          new SpitOutCone(rollersSubsystem)),
        // go to the safe position
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe),
        new ParallelRaceGroup(
          // hold the current position
          new HoldPosition(wristSubsystem, armSubsystem),
          // drive across the community line
          new DriveForwardAuto(driveSubsystem, AutoConstants.distanceForward, false))    
    );
    
  }
}
