// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Conveyor extends SubsystemBase {
    public boolean conveyor1ON = false;
    public boolean conveyor1Reversed = false;
    public boolean conveyor2ON = false;
    public boolean conveyor2Reversed = false;
    /*
    CANSparkMax m_conv1motor;
    CANSparkMax m_conv2motor;
    */
    WPI_VictorSPX m_conv1motor;
    WPI_VictorSPX m_conv2motor;
    public DigitalInput ballsensor1;
    public DigitalInput ballsensor2;

  public Conveyor(){
    try{
      //m_conv1motor = new CANSparkMax(CANBusID.conveyor1, MotorType.kBrushed);
      //m_conv2motor = new CANSparkMax(CANBusID.conveyor2, MotorType.kBrushed);
      m_conv1motor = new WPI_VictorSPX(CANBusID.conveyor1);
      m_conv2motor = new WPI_VictorSPX(CANBusID.conveyor2);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating conveyor motors " + ex.getMessage(), true);
    }
    try{
      ballsensor1 = new DigitalInput(Ports.Conveyor1ballsensor);
      ballsensor2 = new DigitalInput(Ports.Conveyor2ballsensor);
    } catch (RuntimeException ex){
      DriverStation.reportError("Error instantiating ballsensors " + ex.getMessage(), true);
    }
  }

    public void conveyor1ON(){
      m_conv1motor.set(ConveyorC.speed);
      conveyor1ON = true;
    }
    public void conveyor1Reversed(){
      m_conv1motor.set(-ConveyorC.speed);
      conveyor1Reversed = true;
    }

    public void conveyor1OFF(){
      m_conv1motor.set(0.0);
      conveyor1ON = false;
    }

    public void conveyor2ON(){
      m_conv2motor.set(ConveyorC.speed);
      conveyor2ON = true;
    }
    public void conveyor2Reversed(){
      m_conv2motor.set(-ConveyorC.speed);
      conveyor2Reversed = true;
    }

    public void conveyor2OFF(){
      m_conv2motor.set(0.0);
      conveyor2ON = false;
    }

    public void toggleconveyor1(){
      if (conveyor1ON){
        conveyor1OFF();
      }else{
        conveyor1ON();
      }
    }
    public void toggleconveyor2(){
      if (conveyor2ON){
        conveyor2OFF();
      }else{
        conveyor2ON();
      }
    }
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putBoolean("Conveyor On", conveyor1ON);
    } 


    public void DigiConvey1(){
      //Might want to add a debouncer to smooth transition
      // Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
      // So if currently false the signal must go true for at least .1 seconds before being read as a True signal.
      //if (m_debouncer.calculate(input.get())) {
        // Do something now that the DI is True.
      //}
      if (ballsensor1.get()) {
        conveyor1ON();
      } else {
        conveyor1OFF();
      }
  }

  public void DigiConvey2(){
    if (ballsensor2.get()) {
      conveyor2ON();
    } else {
      conveyor2OFF();
    }
}
}