// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import  frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  WPI_VictorSPX m_rightlead = new WPI_VictorSPX(Constants.CANBusID.rightkMotor1);
  WPI_VictorSPX m_rightfollow = new WPI_VictorSPX(Constants.CANBusID.rightkMotor2);
  WPI_VictorSPX m_leftlead = new WPI_VictorSPX(Constants.CANBusID.leftkMotor1);
  WPI_VictorSPX m_leftfollow = new WPI_VictorSPX(Constants.CANBusID.leftkMotor1);
  DifferentialDrive m_drive;
  double MaxOutput = Constants.drive.MaxSpeed;

  public DriveTrain() {
    m_rightlead.setInverted(true);
    m_rightfollow.setInverted(true);
    m_rightfollow.follow(m_rightlead);
    m_leftfollow.follow(m_leftlead);
    m_drive = new DifferentialDrive(m_leftlead, m_rightlead);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setMaxOutput(double maxvalue){
    MaxOutput = maxvalue;
  }
  public void TankDrive(double leftspeed, double rightspeed){
    
    leftspeed = MaxOutput < leftspeed ? MaxOutput : leftspeed ;
    leftspeed = leftspeed < -MaxOutput ? -MaxOutput : leftspeed ;
    rightspeed = MaxOutput < rightspeed ? MaxOutput : rightspeed ;
    rightspeed = rightspeed < -MaxOutput ? -MaxOutput : rightspeed ;
    m_drive.tankDrive(leftspeed, rightspeed);
  }
}