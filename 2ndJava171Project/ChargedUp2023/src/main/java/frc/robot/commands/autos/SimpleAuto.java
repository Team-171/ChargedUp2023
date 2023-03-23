// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** An example command that uses an example subsystem. */
public class SimpleAuto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SimpleAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem) {
    addCommands(
        new SuckInCone(rollersSubsystem),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition, ArmConstants.thirdLevelEncoderPosition),
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem), 
          new SpitOutCone(rollersSubsystem)),
        new ParallelRaceGroup(
          new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe),
          new DriveForwardAuto(driveSubsystem, AutoConstants.distanceForward))    
    );
    
  }
}
