// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Conveyor extends SubsystemBase {
    public boolean conveyorON = false;
    public boolean conveyorReversed = false;
    WPI_VictorSPX m_intakemotor = new WPI_VictorSPX(Constants.CANBusID.dintakemotor);

  public void startFirstIntake(){
    m_intakemotor.set(frc.robot.Constants.firstIntake.firstIntakespeed);
    firstIntakeON = true;
    }
  }
