// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.management.relation.RoleResult;
import javax.swing.text.StyleContext.SmallAttributeSet;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autos.Autos;
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
  private final GearShiftSubsystem gearShiftSubsystem;

  private SendableChooser<Command> autoChooser;

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
    gearShiftSubsystem = new GearShiftSubsystem();

    // set the default command so that this will run constantly
    driveSubsystem.setDefaultCommand(
      new TankDriveCommand(driveSubsystem, () -> -driverController.getLeftY(), () -> -driverController.getRightX()));

    rollersSubsystem.setDefaultCommand(
      new IntakeRollersCommand(rollersSubsystem, () -> operatorController.getRawAxis(OperatorConstants.rightTrigger) - operatorController.getRawAxis(OperatorConstants.leftTrigger)));
    
    armSubsystem.setDefaultCommand(
      new ArmCommand(armSubsystem, () -> operatorController.getLeftY()));
      
    wristSubsystem.setDefaultCommand(
      new WristCommand(wristSubsystem, () -> operatorController.getRightY(), false, false, false, false, false, false));

    // Change the objects to Commands
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Forward Auto", Autos.driveForwardAuto(driveSubsystem));
    autoChooser.addOption("Balance Auto", Autos.balanceAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem));
    autoChooser.setDefaultOption("Simple Auto", Autos.simpleAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem));
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
    operatorController.a().whileTrue(new ConePickupPosition(wristSubsystem, armSubsystem));
    operatorController.b().whileTrue(new Level3ScorePosition(wristSubsystem, armSubsystem));
    operatorController.x().whileTrue(new CubePickupPosition(wristSubsystem, armSubsystem));
    operatorController.y().whileTrue(new ConeLevel2Position(wristSubsystem, armSubsystem));
  
    operatorController.leftBumper().or(operatorController.rightBumper()).whileTrue(new SafePosition(wristSubsystem, armSubsystem));
    operatorController.button(DriveConstants.selectControllerbutton).or(operatorController.button(DriveConstants.startControllerButton)).whileTrue(new ResetPosition(wristSubsystem, armSubsystem));

    driverController.rightBumper().onTrue(new GearShiftCommand(gearShiftSubsystem));
    driverController.x().whileTrue(new BalanceCommand(driveSubsystem));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
