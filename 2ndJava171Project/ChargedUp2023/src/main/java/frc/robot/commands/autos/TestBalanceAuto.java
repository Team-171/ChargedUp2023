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

/** A 21 point auto */
public class TestBalanceAuto extends SequentialCommandGroup {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  /**
   * Creates a new TestBalanceAuto.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TestBalanceAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem,
      IntakeRollersSubsystem rollersSubsystem) {
    addCommands(
        // suck in the cone to secure it
        new SuckInCone(rollersSubsystem),
        // go to the third level shooting position
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition,
            ArmConstants.thirdLevelEncoderPosition),
        new ParallelRaceGroup(
            // hold the current position
            new HoldPosition(wristSubsystem, armSubsystem),
            // score the cone
            new SpitOutCone(rollersSubsystem)),
        // go to the safe position
        new SetPreset(wristSubsystem, armSubsystem, WristConstants.reset, ArmConstants.reset),
        new ParallelRaceGroup(
            // hold the current position
            new HoldPosition(wristSubsystem, armSubsystem),
            // drive up to the middle of the charge station normal
            new DriveForwardAuto(driveSubsystem, AutoConstants.slowBalanceDistance, false)),
        new ParallelRaceGroup(
            // hold the current position
            new HoldPosition(wristSubsystem, armSubsystem),
            // drive past the charge station slowly
            new DriveForwardAuto(driveSubsystem, AutoConstants.crossLineDistance, true)),
        new ParallelRaceGroup(
            // hold the current position
            new HoldPosition(wristSubsystem, armSubsystem),
            // drive back onto the balance station
            new DriveForwardAuto(driveSubsystem, AutoConstants.backDistance, false)),
        new ParallelRaceGroup(
            // hold the current position
            new HoldPosition(wristSubsystem, armSubsystem),
            // balance on the charge station
            new BalanceCommand(driveSubsystem)));

  }
}
