// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {
  // Right Side Motor Controllers

  WPI_VictorSPX m_rightlead = new WPI_VictorSPX(CANBusID.kRightMotor1);
  WPI_VictorSPX m_rightfollow = new WPI_VictorSPX(CANBusID.kRightMotor2);
  MotorControllerGroup m_right = new MotorControllerGroup(m_rightlead, m_rightfollow);

  double maxspeed = DriveC.MaxSpeed;

  // Left Side Motor Controllers
  WPI_VictorSPX m_leftlead = new WPI_VictorSPX(CANBusID.kLeftMotor1);
  WPI_VictorSPX m_leftfollow = new WPI_VictorSPX(CANBusID.kLeftMotor2);
  MotorControllerGroup m_left = new MotorControllerGroup(m_leftlead, m_leftfollow);

  DifferentialDrive m_drive;

  AHRS ahrs;

  public void initMotor() {

    // m_rightfollow.follow(m_rightlead);
    // m_leftfollow.follow(m_leftlead);
    m_left.setInverted(true);
    // m_rightfollow.setInverted(true);
    m_drive = new DifferentialDrive(m_left, m_right);

  }

  public Encoder m_leftencoder = new Encoder(Ports.LeftDriveEncoder1, Ports.LeftDriveEncoder2);
  public Encoder m_rightencoder = new Encoder(Ports.RightDriveEncoder1, Ports.RightDriveEncoder2);

  public DriveTrain() {
    initMotor();
    m_leftencoder.setDistancePerPulse(DriveC.distancePerPulse);
    m_rightencoder.setDistancePerPulse(DriveC.distancePerPulse);
    addChild("Right Encoder", m_rightencoder);
    addChild("Leftt Encoder", m_leftencoder);
    try {
      ahrs = new AHRS(SerialPort.Port.kUSB1);
      ahrs.enableLogging(true);
      addChild("Gyro", ahrs);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }
  }

  public double getAverageDistance() {
    return (m_leftencoder.getDistance() - m_rightencoder.getDistance()) / 2;
  }

  public double getLeftDistance() {
    return (m_leftencoder.getDistance());
  }

  public double getRightDistance() {
    return (-m_rightencoder.getDistance());
  }

  public void resetEncoders() {
    m_leftencoder.reset();
    m_rightencoder.reset();
  }

  public void setMaxOutput(double newMax) {
    maxspeed = newMax;
  }

  public void toggleMaxSpeed() {
    if (maxspeed == DriveC.MaxSpeed) {
      maxspeed = DriveC.reducedMaxSpeed;
    } else {
      maxspeed = DriveC.MaxSpeed;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void TankDrive(double leftspeed, double rightspeed) {
    if (leftspeed > maxspeed) {
      leftspeed = maxspeed;
    }
    if (rightspeed > maxspeed) {
      rightspeed = maxspeed;
    }
    if (leftspeed < -maxspeed) {
      leftspeed = -maxspeed;
    }
    if (rightspeed < -maxspeed) {
      rightspeed = -maxspeed;
    }
    // System.out.printf("leftspeed %f rightspeed %f",leftspeed, rightspeed);
    m_drive.tankDrive(leftspeed, rightspeed);
  }

  public void ArcadeDrive(double forwardspeed, double rotationspeed) {
    if (forwardspeed > maxspeed) {
      forwardspeed = maxspeed;
    }
    if (forwardspeed < -maxspeed) {
      forwardspeed = -maxspeed;
    }
    if (rotationspeed > maxspeed) {
      rotationspeed = maxspeed;
    }
    if (rotationspeed < -maxspeed) {
      rotationspeed = -maxspeed;
    }
    // System.out.println("Drive: " + forwardspeed + ", " + rotationspeed);
    m_drive.arcadeDrive(-forwardspeed, rotationspeed, DriveC.SquareInputs);
  }

  /** Zeroes the heading of the robot. */
  public void resetHeading() {
    ahrs.reset();
  }

  /** Zeroes the displacement of the robot. */
  public void resetDisplacement() {
    ahrs.resetDisplacement();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    try {
      return Math.IEEEremainder(ahrs.getAngle(), 360);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error Getting heading:  " + ex.getMessage(), true);
      return -1000;
    }
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return ahrs.getRate();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Distance", getAverageDistance());
    SmartDashboard.putNumber("Right Encoder Distance", getRightDistance());
    SmartDashboard.putNumber("Left Encoder Distance", getLeftDistance());
    SmartDashboard.putNumber("Gyro", getHeading());
    // This method will be called once per scheduler run
  }
}
