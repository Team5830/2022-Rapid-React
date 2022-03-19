// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class Conveyor extends SubsystemBase {
  public boolean conveyor1ON = false;
  public boolean conveyor1Reversed = false;
  public boolean conveyor2ON = false;
  public boolean conveyor2Reversed = false;
  CANSparkMax m_conv1motor;
  CANSparkMax m_conv2motor;
  public DigitalInput ballsensor1;
  public DigitalInput ballsensor2;
  public boolean ballaway1 = false;
  public boolean ballaway2 = false;
  public boolean ballsensed1 = false;
  public boolean ballsensed2 = false;
  public boolean jammed = false;
  public double targetPosition;
  RelativeEncoder conv2Encoder;
  public SparkMaxPIDController m_pidController;

  public Conveyor() {

    try {
      m_conv1motor = new CANSparkMax(CANBusID.conveyor1, MotorType.kBrushless);
      m_conv1motor.restoreFactoryDefaults();
      m_conv2motor = new CANSparkMax(CANBusID.conveyor2, MotorType.kBrushless);
      m_conv2motor.restoreFactoryDefaults();
      m_conv2motor.setInverted(true);
      m_pidController = m_conv2motor.getPIDController();
      conv2Encoder = m_conv2motor.getEncoder();
      m_pidController.setP(ConveyorC.kP);
      m_pidController.setI(ConveyorC.kI);
      m_pidController.setD(ConveyorC.kD);
      m_pidController.setIZone(ConveyorC.kIz);
      m_pidController.setFF(ConveyorC.kFF);
      m_pidController.setOutputRange(ConveyorC.kMinOutput, ConveyorC.kMaxOutput);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating conveyor motors " + ex.getMessage(), true);
    }
    try {
      ballsensor1 = new DigitalInput(Ports.Conveyor1ballsensor);
      ballsensor2 = new DigitalInput(Ports.Conveyor2ballsensor);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating ballsensors " + ex.getMessage(), true);
    }
  }

  public void conveyor1ON() {
    m_conv1motor.set(ConveyorC.speed);
    conveyor1ON = true;
  }

  public void conveyor1Reversed() {
    m_conv1motor.set(-ConveyorC.speed);
    conveyor1Reversed = true;
  }

  public void conveyor1OFF() {
    m_conv1motor.set(0.0);
    conveyor1ON = false;
  }

  public void conveyor2ON() {
    m_conv2motor.set(ConveyorC.speed);
    conveyor2ON = true;
    jammed = false;
  }

  public void conveyor2Reversed() {
    m_conv2motor.set(-ConveyorC.speed);
    conveyor2Reversed = true;
  }

  public void conveyor2OFF() {
    m_conv2motor.set(0.0);
    conveyor2ON = false;
  }

  public void toggleconveyor1() {
    if (conveyor1ON) {
      conveyor1OFF();
    } else {
      conveyor1ON();
    }
  }

  public void toggleconveyor2() {
    if (conveyor2ON) {
      conveyor2OFF();
    } else {
      conveyor2ON();
    }
  }

  public void conv2down() {
    targetPosition = conv2Encoder.getPosition() - ConveyorC.DownforShot;
    System.out.println("Target: " + targetPosition + ", Current" + conv2Encoder.getPosition());
    m_pidController.setReference(targetPosition, ControlType.kPosition);

  }

  public void conv2up() {
    targetPosition = conv2Encoder.getPosition() + ConveyorC.UpforShot;
    System.out.println("Target: " + targetPosition + ", Current" + conv2Encoder.getPosition());
    m_pidController.setReference(targetPosition, ControlType.kPosition);
  }

  public boolean atTarget() {
    if (Math.abs(targetPosition - conv2Encoder.getPosition()) < 0.1) {
      return true;
    } else {
      return false;
    }
  }

  public void ballJammed() {
    if (conveyor2ON) {
      if (m_conv2motor.getOutputCurrent() >= ConveyorC.currentLimit) {
        jammed = true;
      } else {
        jammed = false;
      }
    } else {
      jammed = false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ballJammed();
    SmartDashboard.putBoolean("Conveyor 1On", conveyor1ON);
    SmartDashboard.putBoolean("Conveyor 2On", conveyor2ON);
    SmartDashboard.putNumber("Conveyor 2 current", m_conv2motor.getOutputCurrent());
    SmartDashboard.putNumber("Conveyor 2 Voltage", m_conv2motor.getVoltageCompensationNominalVoltage());
    SmartDashboard.putNumber("Conveyor 2 Motor", conv2Encoder.getPosition());
    SmartDashboard.putBoolean("Ball Jammed", jammed);
  }

  public void DigiConvey1() {

    Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    if (!m_debouncer.calculate(ballsensor1.get())) {
      conveyor1OFF();
      ballsensed1 = true;
    } else {
      conveyor1ON();
    }
    if (ballsensed1 && m_debouncer.calculate(ballsensor1.get())) {
      ballaway1 = true;
      ballsensed1 = false;
    }

  }

  public void DigiConvey2() {
    Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    if (!m_debouncer.calculate(ballsensor2.get())) {
      conveyor2OFF();
      ballsensed2 = true;
    } else {
      conveyor2ON();
    }
    if (ballsensed2 && m_debouncer.calculate(ballsensor2.get())) {
      ballaway2 = true;
      ballsensed2 = false;
    }
  }
}