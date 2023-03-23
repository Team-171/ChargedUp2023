// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

/** An example command that uses an example subsystem. */
public class ConePickupPosition extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final WristSubsystem wristSubsystem;
  private final ArmSubsystem armSubsystem;
  private boolean armFinished;
  private boolean wristFinished;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ConePickupPosition(WristSubsystem subsystem, ArmSubsystem armSubsystem) {
    wristSubsystem = subsystem;
    this.armSubsystem = armSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armFinished = armSubsystem.moveArm(0, true, false, false, false, false, false, false, false);
    wristFinished = wristSubsystem.moveWrist(0, true, false, false, false, false, false, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.moveArm(0, false, false, false, false, false, false, false, false);
    wristSubsystem.moveWrist(0, false, false, false, false, false, false, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(armFinished && wristFinished)
      return true;
    return false;
  }
}
