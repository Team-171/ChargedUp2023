// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import javax.swing.text.StyledEditorKit.BoldAction;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** An example command that uses an example subsystem. */
public class SetPreset extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final WristSubsystem wristSubsystem;
  private final ArmSubsystem armSubsystem;
  private boolean wristFinished;
  private boolean armFinished;

  private double armPosition;
  private double wristPosition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetPreset(WristSubsystem subsystem, ArmSubsystem armSubsystem, double wristPosition, double armPosition) {
    wristSubsystem = subsystem;
    this.armSubsystem = armSubsystem;
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
    armFinished = armSubsystem.moveArmButton(armPosition);
    wristFinished = wristSubsystem.moveWristButton(wristPosition);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(armFinished && wristFinished)
      return true;
    return false;
  }
}
