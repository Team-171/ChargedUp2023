// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class GearShiftSubsystem extends SubsystemBase {
  // Initialization of variables
  DoubleSolenoid solenoid;
  Compressor compressor;

  
  /** 
   * Creates a new GearShiftSubsystem.
   * Used to shift gears 
  */
  public GearShiftSubsystem() {
    // Creates a double solenoid to shift between low and high gear
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticConstants.forwardChannel, PneumaticConstants.reverseChannel);
    
    // Sets the drive to low gear
    solenoid.set(Value.kReverse);

    // Creates and enables the pneumatic compressor
    compressor = new Compressor(PneumaticConstants.moduleID,PneumaticsModuleType.REVPH);
    compressor.enableAnalog(PneumaticConstants.minPressure, PneumaticConstants.maxPressure);

  }

  /**
   * Toggles from low to high gear
   */
  public void togglePneumatic(){
     solenoid.toggle();
     SmartDashboard.putString("Solenoid: ", solenoid.get().name());
  }

  @Override
  public void periodic() {
    // SmartDashboard posting
    // Put SmartDashboard in periodic so the output is always working even when the robot is disabled

    SmartDashboard.putNumber("Compressor Pressure: ", compressor.getPressure());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
