// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class FirstIntake extends SubsystemBase {
  public boolean firstIntakeON = false;
  public boolean firstIntakeReversed = false;
  //CanSparkMax m_intakemotor = new CanSparkMax(CANBusID.dintakemotor);
  CANSparkMax m_intakemotor;
  CANSparkMax m_exotor;
  SparkMaxPIDController m_pidController;
  RelativeEncoder m_encoder;
  boolean isExtended = false;
  public FirstIntake(){
    try{
      m_intakemotor = new CANSparkMax(CANBusID.dintakemotor, MotorType.kBrushless);
      m_exotor  = new CANSparkMax(CANBusID.dexotor, MotorType.kBrushless);
      m_exotor.restoreFactoryDefaults();
      m_intakemotor.restoreFactoryDefaults();
      m_pidController= m_exotor.getPIDController();
      m_encoder = m_exotor.getEncoder();
    } catch (RuntimeException ex) {
      DriverStation.reportError("error loading failed" + ex.getMessage(), true);
    }
    // set PID coefficients
    m_pidController.setP(firstIntake.P);
    m_pidController.setI(firstIntake.I);
    m_pidController.setD(firstIntake.D);
    m_pidController.setIZone(firstIntake.zI);
    m_pidController.setFF(firstIntake.F);
    m_pidController.setOutputRange(firstIntake.kMinOutput, firstIntake.kMaxOutput);
    }
    
  public void extendIntake(){
   if (!isExtended){
    m_pidController.setReference(firstIntake.ExtendDistance, ControlType.kPosition);
    isExtended = true;
   }
  }
  
  public void retractIntake(){
    if (isExtended){
      m_pidController.setReference(0.0, ControlType.kPosition);
      isExtended = false;
    }
  }

  public void toggleExtension(){
    if (isExtended ){
      retractIntake();
    }else{
      extendIntake();
    }
  }

  public void startFirstIntake(){
    m_intakemotor.set(firstIntake.firstIntakespeed);
    firstIntakeON = true;
  }

  public void reverseFirstIntake(){
    m_intakemotor.set(-firstIntake.firstIntakespeed);
    firstIntakeON = true;
    firstIntakeReversed = true;
  }

  public void stopFirstIntake(){
    m_intakemotor.set(0);
    firstIntakeON = false;
    firstIntakeReversed = false;
  }
  //Call this to toggle the intake 
  public void toggleFirstIntake(){
    if (firstIntakeON){
      stopFirstIntake();
    }else{
      startFirstIntake();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("FirstIntakeOn", firstIntakeON);
    SmartDashboard.putBoolean("FirstIntakeReversed", firstIntakeReversed);
  }
}
