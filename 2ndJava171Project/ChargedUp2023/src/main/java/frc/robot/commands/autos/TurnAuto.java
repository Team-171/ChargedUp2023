// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDriveSubsystem;


/** A command to turn the robot automatically.
 *  NEVER TESTED!
 */
public class TurnAuto extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TankDriveSubsystem driveSubsystem;
  
  // amount to turn the robot in degrees
  private double degrees;
  // is the robot turned to the target degree
  private boolean finished;
  // is the robot turning slowly
  private boolean slow;

  /**
   * Creates a new TurnAuto command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnAuto(TankDriveSubsystem subsystem, double degrees, boolean slow) {
    driveSubsystem = subsystem;
    this.degrees = degrees;
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
    // Turn the robot to target degrees. Turn the robot slowly if it should be slow.
    finished = driveSubsystem.turn(degrees, slow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // set the motors to 0
    driveSubsystem.arcadeDrive(0, 0, false);
    // reset the gyro
    driveSubsystem.resetYaw();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if the robot has turned to the target degrees
    if (finished)
      return true;
    return false;
  }
}
