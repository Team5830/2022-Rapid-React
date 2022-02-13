// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FirstIntake extends SubsystemBase {
  public boolean firstIntakeON = false;
  public boolean firstIntakeReversed = false;
  WPI_VictorSPX m_intakemotor = new WPI_VictorSPX(Constants.CANBusID.dintakemotor);
  CANSparkMax m_exotor = new CANSparkMax(Constants.CANBusID.dexotor, MotorType.kBrushless);
  SparkMaxPIDController m_pidController = m_exotor.getPIDController();
  RelativeEncoder m_encoder = m_exotor.getEncoder();
  boolean isExtended = false;
  public FirstIntake(){
    try{
      m_exotor.restoreFactoryDefaults();
    } catch (RuntimeException ex) {
      DriverStation.reportError("error loading failed" + ex.getMessage(), true);
    }
    // set PID coefficients
    m_pidController.setP(Constants.firstIntake.P);
    m_pidController.setI(Constants.firstIntake.I);
    m_pidController.setD(Constants.firstIntake.D);
    m_pidController.setIZone(Constants.firstIntake.zI);
    m_pidController.setFF(Constants.firstIntake.F);
    m_pidController.setOutputRange(Constants.firstIntake.kMinOutput, Constants.firstIntake.kMaxOutput);
    }
    
  public void extendIntake(){
   if (!isExtended){
    m_pidController.setReference(Constants.firstIntake.ExtendDistance, ControlType.kPosition);
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
    m_intakemotor.set(frc.robot.Constants.firstIntake.firstIntakespeed);
    firstIntakeON = true;
  }

  public void reverseFirstIntake(){
    m_intakemotor.set(-frc.robot.Constants.firstIntake.firstIntakespeed);
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
