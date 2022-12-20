// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class Pneumatics extends SubsystemBase {
  Compressor pcmCompressor; 
  Solenoid solenoidPCM1;
  DoubleSolenoid doublePCM23;
  public Pneumatics() {
    try {
      pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
      solenoidPCM1 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
      doublePCM23 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
      pcmCompressor.enableDigital();
      doublePCM23.set(Value.kReverse);
    } catch (RuntimeException ex) {
      DriverStation.reportError("error loading failed" + ex.getMessage(), true);
    }
  }
  
  public void toggleSingle(){
    solenoidPCM1.toggle();
  }
  
  public void toggleDouble(){
    doublePCM23.toggle();
  }

  public void setSingleON(){
    solenoidPCM1.set(true);
  }

  public void setSingleOFF(){
    solenoidPCM1.set(false);
  }

  public double getPressure() {
    return (pcmCompressor.getPressure());
  }

  public double getCurrent() {
    return (pcmCompressor.getCurrent());
  }

  public boolean getSwitch(){
    return (pcmCompressor.getPressureSwitchValue());
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Compressor On", getSwitch());
    SmartDashboard.putNumber("Compressor Current", getCurrent());
    SmartDashboard.putNumber("Compressor Pressure", getPressure());
    }

}


