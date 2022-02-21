// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
/*
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
*/
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class FirstIntake extends SubsystemBase {
  public boolean firstIntakeON = false;
  public boolean firstIntakeReversed = false;
  /*
  CANSparkMax m_intakemotor;
  CANSparkMax m_exotor;
  SparkMaxPIDController m_pidController;
  */
  WPI_VictorSPX m_intakemotor;
  WPI_VictorSPX m_exotor;
  PIDController m_pidController;
  //RelativeEncoder m_encoder;
  Encoder m_encoder;
  boolean isExtended = false;
  public FirstIntake(){
    try{
      m_intakemotor = new WPI_VictorSPX(CANBusID.dintakemotor); //new CANSparkMax(CANBusID.dintakemotor, MotorType.kBrushless);
      m_exotor  = new WPI_VictorSPX(CANBusID.dexotor);         //new CANSparkMax(CANBusID.dexotor, MotorType.kBrushless);
      Encoder m_encoder = new Encoder(5, 6); //Temporary
      m_pidController = new PIDController(MovePID.P, MovePID.I, MovePID.D);
      //m_exotor.restoreFactoryDefaults();
      //m_intakemotor.restoreFactoryDefaults();
      //m_pidController= m_exotor.getPIDController();
      //m_encoder = m_exotor.getEncoder();
      
    } catch (RuntimeException ex) {
      DriverStation.reportError("error loading failed" + ex.getMessage(), true);
    }
    // set PID coefficients
    /*
    m_pidController.setP(firstIntake.P);
    m_pidController.setI(firstIntake.I);
    m_pidController.setD(firstIntake.D);
    m_pidController.setIZone(firstIntake.zI);
    m_pidController.setFF(firstIntake.F);
    m_pidController.setOutputRange(firstIntake.kMinOutput, firstIntake.kMaxOutput);
    */
    }
    
  public void extendIntake(){
   if (!isExtended){
    //m_pidController.setReference(firstIntake.ExtendDistance, ControlType.kPosition);
    m_pidController.setSetpoint(firstIntake.ExtendDistance);
    isExtended = true;
   }
  }
  
  public void retractIntake(){
    if (isExtended){
      //m_pidController.setReference(0.0, ControlType.kPosition);
      m_pidController.setSetpoint(0.0);
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
