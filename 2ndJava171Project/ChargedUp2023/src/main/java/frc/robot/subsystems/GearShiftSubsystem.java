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

  DoubleSolenoid solenoid;
  Compressor compressor;

  
  /** Creates a new ExampleSubsystem. */
  public GearShiftSubsystem() {
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticConstants.forwardChannel, PneumaticConstants.reverseChannel);
    solenoid.set(Value.kReverse);

    compressor = new Compressor(PneumaticConstants.moduleID,PneumaticsModuleType.REVPH);
    compressor.enableAnalog(PneumaticConstants.minPressure, PneumaticConstants.maxPressure);

  }

  public void togglePneumatic(){
     solenoid.toggle();
     SmartDashboard.putString("Solenoid: ", solenoid.get().name());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Compressor Pressure: ", compressor.getPressure());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}