// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDriveSubsystem;


public class DriveForwardAuto extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TankDriveSubsystem driveSubsystem;
  // Distance to drive forward
  private double distance;
  // If the robot reached the distance
  private boolean finished;
  // If the robot should drive slow
  private boolean slow;

  /**
   * Creates the drive forward auto
   * Drives forward a set distance
   * @param distance double The distance to drive
   * @param slow boolean If the robot should drive slower
   */
  public DriveForwardAuto(TankDriveSubsystem subsystem, double distance, boolean slow) {
    driveSubsystem = subsystem;
    this.distance = distance;
    this.slow = slow;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drives forward and returns if it reached the destination
    finished = driveSubsystem.driveForward(distance, slow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // When the robot is at the destination, set the speed to 0
    driveSubsystem.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finished)
      return true;
    return false;
  }
}
