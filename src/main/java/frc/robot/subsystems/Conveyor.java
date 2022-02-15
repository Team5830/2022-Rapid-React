// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
public class Conveyor extends SubsystemBase {
    public boolean conveyor1ON = false;
    public boolean conveyor1Reversed = false;
    public boolean conveyor2ON = false;
    public boolean conveyor2Reversed = false;
    WPI_VictorSPX m_conv1motor;
    WPI_VictorSPX m_conv2motor;

  public Conveyor(){
    try{
      m_conv1motor = new WPI_VictorSPX(CANBusID.conveyor1);
      m_conv2motor = new WPI_VictorSPX(CANBusID.conveyor2);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating conveyor motors " + ex.getMessage(), true);
    }
  }

    public void conveyor1ON(){
      m_conv1motor.set(ConveyorC.speed);
      conveyor1ON = true;
    }

    public void conveyor1OFF(){
      m_conv1motor.set(0.0);
      conveyor1ON = false;
    }

    public void conveyor2ON(){
      m_conv2motor.set(ConveyorC.speed);
      conveyor2ON = true;
    }

    public void conveyor2OFF(){
      m_conv2motor.set(0.0);
      conveyor2ON = false;
    }

    public void toggleconveyor(){
      if (conveyor1ON && conveyor2ON){
        conveyor1OFF();
        conveyor2OFF();
      }else{
        conveyor1ON();
        conveyor2ON();
      }
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putBoolean("Conveyor On", conveyor1ON);
    }
  
  }
