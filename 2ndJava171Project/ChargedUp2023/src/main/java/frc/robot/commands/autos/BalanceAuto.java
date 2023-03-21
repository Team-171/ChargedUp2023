// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import frc.robot.commands.*;


/** An example command that uses an example subsystem. */
public class BalanceAuto extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BalanceAuto(TankDriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeRollersSubsystem rollersSubsystem) {
    addCommands(
        new SuckInCone(rollersSubsystem),
        new Level3ScorePosition(wristSubsystem, armSubsystem),
        new ParallelRaceGroup(
          new HoldPosition(wristSubsystem, armSubsystem), 
          new SpitOutCone(rollersSubsystem)),
        new ParallelRaceGroup(
          new SafePosition(wristSubsystem, armSubsystem),
          new DriveForwardAuto(driveSubsystem, AutoConstants.balanceDistanceForward)),
        new ParallelRaceGroup(
            new CubePickupPosition(wristSubsystem, armSubsystem),
            new BalanceCommand(driveSubsystem)
        )
    );
    
  }
}
