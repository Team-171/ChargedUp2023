// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;


/** A command that sets the arm and wrist to preset values. */
public class SetPreset extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final WristSubsystem wristSubsystem;
  private final ArmSubsystem armSubsystem;

  // has the wrist reached its target
  private boolean wristFinished;
  // has the arm reached its target
  private boolean armFinished;

  // preset value for the wrist
  private double wristPosition;
  //preset value for the arm
  private double armPosition;

  /**
   * Creates a new SetPreset command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetPreset(WristSubsystem subsystem, ArmSubsystem armSubsystem, double wristPosition, double armPosition) {
    wristSubsystem = subsystem;
    this.armSubsystem = armSubsystem;
    // set the targets equal to the passed values
    this.armPosition = armPosition;
    this.wristPosition = wristPosition;

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
    // Move the arm to the target. Return true if it has reached the target.
    armFinished = armSubsystem.moveArmButton(armPosition);
    // Move the wrist to the target. Return true if it has reached the target.
    wristFinished = wristSubsystem.moveWristButton(wristPosition);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if the arm and the wrist have reached their respective targets
    if(armFinished && wristFinished)
      return true;
    return false;
  }
}
