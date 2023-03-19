// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** An example command that uses an example subsystem. */
public class WristCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final WristSubsystem wristSubsystem;
  private DoubleSupplier speed;
  boolean aButton;
  boolean bButton;
  boolean xButton;
  boolean yButton;
  boolean reset;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristCommand(WristSubsystem subsystem, DoubleSupplier speed, boolean aButton, boolean bButton, boolean xButton, boolean yButton, boolean reset) {
    wristSubsystem = subsystem;
    this.speed = speed;
    this.aButton = aButton;
    this.bButton = bButton;
    this.xButton = xButton;
    this.yButton = yButton;
    this.reset = reset;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristSubsystem.moveWrist(speed.getAsDouble(), aButton, bButton, xButton, yButton, reset);

    SmartDashboard.putNumber("Wrist Encoder Distance: ", wristSubsystem.getWristEncoder());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
