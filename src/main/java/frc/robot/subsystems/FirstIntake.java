// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.firstIntake.*;

public class FirstIntake extends SubsystemBase {
    private boolean firstIntakeON =false;
    private boolean FirstIntakeReversed =false;
public void startFirstIntake(){
  firstIntakespeed =(Constants.firstIntake.firstIntakespeed);
  firstIntakeON = true;
  SmartDashboard.putBoolean("FirstIntakeOn", firstIntakeON);
}
public void reverseFirstIntake(){
  FirstIntake.set(0.5);
  FirstIntake.firstIntakeON = true;
  FirstIntake.firstINtakeReversed = true;
  SmartDashboard.putBoolean("FirstIntakeOn", firstIntakeON);
  SmartDashboard.putBoolean("FirstIntakeReversed", firstINtakeReversed);
}
public void stopFirstIntake(){
  firstIntake.set(0);
  FirstIntake.firstIntakeON = false;
  FirstIntake.firstINtakeReversed = false;
  SmartDashboard.putBoolean("FirstIntakeOn", firstIntakeON);
  SmartDashboard.putBoolean("FirstIntakeReversed", firstINtakeReversed);
  }
}