// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.IntakeRollersCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeRollersSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TankDriveSubsystem driveSubsystem;
  private final WristSubsystem wristSubsystem;
  private final IntakeRollersSubsystem rollersSubsystem;
  private final ArmSubsystem armSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(DriveConstants.kDriverControllerPort);

  private final CommandXboxController operatorController = 
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // establish the driving subsystem
    driveSubsystem = new TankDriveSubsystem();
    wristSubsystem = new WristSubsystem();
    rollersSubsystem = new IntakeRollersSubsystem();
    armSubsystem = new ArmSubsystem();

    // set the default command so that this will run constantly
    driveSubsystem.setDefaultCommand(
      new TankDriveCommand(driveSubsystem, () -> -driverController.getLeftY(), () -> driverController.getRightX()));

    rollersSubsystem.setDefaultCommand(
      new IntakeRollersCommand(rollersSubsystem, () -> operatorController.getRawAxis(2) - operatorController.getRawAxis(3)));
    
    armSubsystem.setDefaultCommand(
      new ArmCommand(armSubsystem, () -> operatorController.getLeftY(), operatorController.getHID().getAButtonPressed(), operatorController.getHID().getBButtonPressed(), operatorController.getHID().getXButtonPressed(), operatorController.getHID().getYButtonPressed()));
      
    wristSubsystem.setDefaultCommand(
      new WristCommand(wristSubsystem, () -> operatorController.getRightY(), false, false, false, false));
      
    // Configure the trigger bindings
    configureBindings();
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    operatorController.a().whileTrue(new ArmCommand(armSubsystem, () -> 0, true, false, false, false));
    operatorController.b().whileTrue(new ArmCommand(armSubsystem, () -> 0, false, true, false, false));
    operatorController.x().whileTrue(new ArmCommand(armSubsystem, () -> 0, false, false, true, false));
    operatorController.y().whileTrue(new ArmCommand(armSubsystem, () -> 0, false, false, false, true));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
