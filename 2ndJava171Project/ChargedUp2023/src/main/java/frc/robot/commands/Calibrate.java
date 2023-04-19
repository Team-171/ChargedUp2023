// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TankDriveSubsystem;
 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command to callibrate the gyro */
public class Calibrate extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TankDriveSubsystem driveSubsystem;
  /**
   * Creates a new Calibrate command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Calibrate(TankDriveSubsystem subsystem) {
    driveSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // calibrate the gyro
    driveSubsystem.setup();

    SmartDashboard.putBoolean("AHRS Set Up: ", TankDriveSubsystem.setup);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
