// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
  private final PneumaticSubsystem pneumaticSubsystem;
  private SendableChooser<String> autoChooser;

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
    pneumaticSubsystem = new PneumaticSubsystem();

    // set the default command so that this will run constantly
    driveSubsystem.setDefaultCommand(
      new TankDriveCommand(driveSubsystem, () -> -driverController.getLeftY(), () -> -driverController.getRightX()));

    rollersSubsystem.setDefaultCommand(
      new IntakeRollersCommand(rollersSubsystem, () -> operatorController.getRawAxis(OperatorConstants.rightTrigger) - operatorController.getRawAxis(OperatorConstants.leftTrigger)));
    
    armSubsystem.setDefaultCommand(
      new ArmCommand(armSubsystem, () -> operatorController.getLeftY(), false, false, false, false, false));
      
    wristSubsystem.setDefaultCommand(
      new WristCommand(wristSubsystem, () -> operatorController.getRightY(), false, false, false, false, false));

    pneumaticSubsystem.setDefaultCommand(
      new PneumaticCommand(pneumaticSubsystem, false));
    
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Auto 1", "Auto 1");
    autoChooser.setDefaultOption("Auto 2 (Default)", "Auto 2");
    SmartDashboard.putData(autoChooser);
    
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
    operatorController.a().whileTrue(new ArmCommand(armSubsystem, () -> 0, true, false, false, false, false)).whileTrue(new WristCommand(wristSubsystem, () -> 0, true, false, false, false, false));
    operatorController.b().whileTrue(new ArmCommand(armSubsystem, () -> 0, false, true, false, false, false)).whileTrue(new WristCommand(wristSubsystem, () -> 0, false, true, false, false, false));
    operatorController.x().whileTrue(new ArmCommand(armSubsystem, () -> 0, false, false, true, false, false)).whileTrue(new WristCommand(wristSubsystem, () -> 0, false, false, true, false, false));
    operatorController.y().whileTrue(new ArmCommand(armSubsystem, () -> 0, false, false, false, true, false)).whileTrue(new WristCommand(wristSubsystem, () -> 0, false, false, false, true, false));
  
    operatorController.leftBumper().and(operatorController.rightBumper()).whileTrue(new ArmCommand(armSubsystem, () -> 0, false, false, false, false, true)).whileTrue(new WristCommand(wristSubsystem, () -> 0, false, false, false, false, true));
    
    // driverController.rightBumper().onTrue(new PneumaticCommand(pneumaticSubsystem, true));
    driverController.a().onTrue(new PneumaticCommand2(pneumaticSubsystem, true));
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
