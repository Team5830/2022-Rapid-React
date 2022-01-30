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

public class DriveTrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  // Right Side Motor Controllers
  WPI_VictorSPX m_rightlead = new WPI_VictorSPX(Constants.CANBusID.kRightMotor1);
  WPI_VictorSPX m_rightfollow = new WPI_VictorSPX(Constants.CANBusID.kRightMotor2);

  double maxspeed=Constants.Drive.MaxSpeed;

  // Left Side Motor Controllers
  WPI_VictorSPX m_leftlead = new WPI_VictorSPX(Constants.CANBusID.kLeftMotor1);
  WPI_VictorSPX m_leftfollow = new WPI_VictorSPX(Constants.CANBusID.kLeftMotor2);

  DifferentialDrive m_drive;// = new DifferentialDrive(m_leftlead, m_rightlead);

  public void initMotor() {

    m_rightfollow.follow(m_rightlead);
    m_leftfollow.follow(m_leftlead);
    m_drive = new DifferentialDrive(m_leftlead, m_rightlead);
  }

  public Encoder m_leftencoder = new Encoder(Constants.Ports.LeftDriveEncoder1, Constants.Ports.LeftDriveEncoder2);
  public Encoder m_rightencoder = new Encoder(Constants.Ports.RightDriveEncoder1, Constants.Ports.RightDriveEncoder2);

  public DriveTrain() {
    m_leftencoder.setDistancePerPulse(Constants.Drive.distancePerPulse);
    m_rightencoder.setDistancePerPulse(Constants.Drive.distancePerPulse);
  }
  public double getAverageDistance(){
    return (m_leftencoder.getDistance()-m_rightencoder.getDistance())/2;
    }
    public double getLeftDistance(){
      return (m_leftencoder.getDistance());
    }
    public double getRightDistance(){
      return (-m_rightencoder.getDistance());
    }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Distance", getAverageDistance());
    SmartDashboard.putNumber("Right Encoder Distance",getRightDistance());
    SmartDashboard.putNumber("Left Encoder Distance",getLeftDistance());
    // This method will be called once per scheduler run
  }
  public void setMaxOutput(double newMax) {
    maxspeed=newMax;
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void TankDrive(double leftspeed, double rightspeed){  
    if (leftspeed>maxspeed ){
    leftspeed=maxspeed;
    }
    if (rightspeed>maxspeed ){
      rightspeed=maxspeed;
    }
    if (leftspeed<-maxspeed ){
      leftspeed=-maxspeed;
    }
    if (rightspeed<-maxspeed ){
      rightspeed=-maxspeed;
    }   
    m_drive.tankDrive(leftspeed, rightspeed, Constants.Drive.SquareInputs);
  }

  public void ArcadeDrive(double fowardspeed, double rotationsspeed) {
    m_drive.arcadeDrive(fowardspeed, rotationsspeed, Constants.Drive.SquareInputs);
  }
  
}
