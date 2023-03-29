// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      new TankDriveCommand(driveSubsystem, () -> -driverController.getLeftY(), () -> -driverController.getRightX(), false));

    rollersSubsystem.setDefaultCommand(
      new IntakeRollersCommand(rollersSubsystem, () -> driverController.getRawAxis(DriveConstants.rightTrigger) - driverController.getRawAxis(DriveConstants.leftTrigger)));
    
    // essentially hold the position for the arm
    armSubsystem.setDefaultCommand(
      new ArmCommand(armSubsystem, () -> 0));
    
    // essentially hold the position for the wrist
    wristSubsystem.setDefaultCommand(
      new WristCommand(wristSubsystem, () -> 0));

    // // temp for calibrating presets
    // armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem, () -> operatorController.getLeftY()));

    // wristSubsystem.setDefaultCommand(new WristCommand(wristSubsystem, () -> operatorController.getRightY()));

    // Change the objects to Commands
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Forward Auto", Autos.driveForwardAuto(driveSubsystem));
    autoChooser.addOption("Balance Auto", Autos.balanceAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem));
    autoChooser.addOption("21 Point Balance Auto", Autos.testBalanceAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem));
    autoChooser.addOption("Double Cone Auto", Autos.doubleConeAuto(driveSubsystem, wristSubsystem, armSubsystem, rollersSubsystem));
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
    operatorController.a().whileTrue(new SetPreset(wristSubsystem, armSubsystem, WristConstants.conePickupEncoderPosition, ArmConstants.conePickupEncoderPosition));
    operatorController.x().whileTrue(new SetPreset(wristSubsystem, armSubsystem, WristConstants.cubePickupEncoderPosition, ArmConstants.cubePickupEncoderPostion));
    operatorController.pov(90).whileTrue(new SetPreset(wristSubsystem, armSubsystem, WristConstants.secondLevelEncoderPosition, ArmConstants.secondLevelEncoderPosition));
    operatorController.pov(270).whileTrue(new SetPreset(wristSubsystem, armSubsystem, WristConstants.secondLevelEncoderPosition, ArmConstants.secondLevelEncoderPosition));
    operatorController.y().whileTrue(new SetPreset(wristSubsystem, armSubsystem, WristConstants.inputCubeEncoderPosition, ArmConstants.inputCubeEncoderPosition));
  
    operatorController.button(DriveConstants.selectControllerbutton).or(operatorController.button(DriveConstants.startControllerButton)).whileTrue(new SetPreset(wristSubsystem, armSubsystem, WristConstants.reset, ArmConstants.reset));

    driverController.y().whileTrue(new SetPreset(wristSubsystem, armSubsystem, WristConstants.thirdLevelEncoderPosition, ArmConstants.thirdLevelEncoderPosition));
    driverController.b().whileTrue(new SetPreset(wristSubsystem, armSubsystem, WristConstants.inputConeEncoderPosition, ArmConstants.inputConeEncoderPosition));
    driverController.a().whileTrue(new SetPreset(wristSubsystem, armSubsystem, WristConstants.safe, ArmConstants.safe));

    driverController.rightBumper().onTrue(new GearShiftCommand(gearShiftSubsystem));
    driverController.x().whileTrue(new BalanceCommand(driveSubsystem));
    driverController.leftBumper().whileTrue(new TankDriveCommand(driveSubsystem, () -> -driverController.getLeftY(), () -> -driverController.getRightX(), true));

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
