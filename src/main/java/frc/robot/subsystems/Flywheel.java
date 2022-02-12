// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
 CANSparkMax m_leftlead = new CANSparkMax(Constants.CANBusID.kLeftFlywheel, MotorType.kBrushless);
 CANSparkMax m_rightfollow = new CANSparkMax(Constants.CANBusID.kRightFlywheel, MotorType.kBrushless);
 SparkMaxPIDController m_pidController = m_leftlead.getPIDController();
 RelativeEncoder m_encoder = m_leftlead.getEncoder();
 boolean isshooteron = false;

 public class PidVals {
  public double kP = 0.0012;
  public double kI = 0.000006;
  public double kD = 0.001;
  public double kIz = 0;
  public double kFF = 0;
  public double kMaxOutput = 0.7;
  public double kMinOutput = -0.7;
  public double motorspeed = Constants.Flywheel.shootermotorspeed;
 }

 PidVals oldPidVals = new PidVals();
 public PidVals pidVals = new PidVals();

 public double maxRPM, maxVel, minVel, maxAcc, allowedErr;
 public double motorspeed, velocity_out;

 public Flywheel() {
  try {
   m_leftlead.restoreFactoryDefaults();
   m_rightfollow.restoreFactoryDefaults();
   m_leftlead.follow(ExternalFollower.kFollowerDisabled, 0);
   m_rightfollow.follow(m_leftlead, true);
  } catch (RuntimeException ex) {
   DriverStation.reportError("error loading failed" + ex.getMessage(), true);
  }

  // set PID coefficients
  m_pidController.setP(pidVals.kP);
  m_pidController.setI(pidVals.kI);
  m_pidController.setD(pidVals.kD);
  m_pidController.setIZone(pidVals.kIz);
  m_pidController.setFF(pidVals.kFF);
  m_pidController.setOutputRange(pidVals.kMinOutput, pidVals.kMaxOutput);
 }

 @Override
 public void periodic() {
  // if PID coefficients have changed, write new values to controller
  if((oldPidVals.kP != pidVals.kP)) { m_pidController.setP(pidVals.kP); oldPidVals.kP = pidVals.kP; }
  if((oldPidVals.kI != pidVals.kI)) { m_pidController.setI(pidVals.kI); oldPidVals.kI = pidVals.kI; }
  if((oldPidVals.kD != pidVals.kD)) { m_pidController.setD(pidVals.kD); oldPidVals.kD = pidVals.kD; }
  if((oldPidVals.kIz != pidVals.kIz)) { m_pidController.setIZone(pidVals.kIz); oldPidVals.kIz = pidVals.kIz; }
  if((oldPidVals.kFF != pidVals.kFF)) { m_pidController.setFF(pidVals.kFF); oldPidVals.kFF = pidVals.kFF; }
  if (isshooteron){
   if((motorspeed==0)){motorspeed = Constants.Flywheel.shootermotorspeed;}
   m_pidController.setReference(motorspeed, ControlType.kVelocity);
  }else{
   shooteroff();
   //m_pidController.setReference(0, ControlType.kVelocity);
  }
  if((oldPidVals.kMaxOutput != pidVals.kMaxOutput) || (oldPidVals.kMinOutput != pidVals.kMinOutput)) { 
   m_pidController.setOutputRange(pidVals.kMinOutput, pidVals.kMaxOutput); 
   oldPidVals.kMaxOutput = pidVals.kMaxOutput; oldPidVals.kMinOutput = pidVals.kMinOutput; 
  }
  try {
   velocity_out = m_encoder.getVelocity();
  } catch (RuntimeException ex){
   DriverStation.reportError("Shooter: Not able to get velocity " + ex.getMessage(),true);
   velocity_out = 0;
  }
 }

 public boolean getShooterState() {
  return isshooteron;
 }

 public void shooteron() {
  m_pidController.setReference(motorspeed, ControlType.kVelocity);
  isshooteron = true;
 }

 public void shooteroff() {
  m_leftlead.set(0);
  isshooteron = false;
 }

 public boolean readyToShoot() {
  return (motorspeed - velocity_out < 1000);
 }
}
