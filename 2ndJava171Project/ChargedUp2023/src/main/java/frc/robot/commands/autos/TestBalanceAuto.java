// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.SetPreset;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeRollersSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.WristSubsystem;


/** An example command that uses an example subsystem. */
public class TestBalanceAuto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestBalanceAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem) {
    addCommands(
        new SuckInCone(rollersSubsystem),
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition, ArmConstants.thirdLevelEncoderPosition),
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem), 
          new SpitOutCone(rollersSubsystem)),
        new ParallelRaceGroup(
          new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe),
          new DriveForwardAuto(driveSubsystem, AutoConstants.crossLineDistance)),
        new ParallelRaceGroup(
          new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe),
          new DriveForwardAuto(driveSubsystem, AutoConstants.backDistance)),
        new ParallelRaceGroup(
          new SetPreset(wristSubsystem, armSubsystem, WristConstants.cubePickupEncoderPosition, ArmConstants.cubePickupEncoderPostion),
            new BalanceCommand(driveSubsystem)
        )
    );
    
  }
}
