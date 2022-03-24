// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {
  
  double maxspeed = DriveC.MaxSpeed;
  DifferentialDrive m_drive;
  private DifferentialDriveOdometry m_odometry;
  private DifferentialDriveKinematics m_kinematics;
  private SimpleMotorFeedforward m_feedforward;
  AHRS ahrs;
  private PIDController m_leftPIDController; 
  private PIDController m_rightPIDController;
  public Encoder m_leftencoder;
  public Encoder m_rightencoder;
  MotorControllerGroup m_right;
  MotorControllerGroup m_left;
  private Field2d m_field = new Field2d();
  public PowerDistribution m_powerdist;
  public DriveTrain() {
    try{
      // Right Side Motor Controllers
      WPI_VictorSPX m_rightlead = new WPI_VictorSPX(CANBusID.kRightMotor1);
      WPI_VictorSPX m_rightfollow = new WPI_VictorSPX(CANBusID.kRightMotor2);
      MotorControllerGroup m_right = new MotorControllerGroup(m_rightlead, m_rightfollow);
      // Left Side Motor Controllers
      WPI_VictorSPX m_leftlead = new WPI_VictorSPX(CANBusID.kLeftMotor1);
      WPI_VictorSPX m_leftfollow = new WPI_VictorSPX(CANBusID.kLeftMotor2);
      MotorControllerGroup m_left = new MotorControllerGroup(m_leftlead, m_leftfollow);
      m_right.setInverted(true);
      m_drive = new DifferentialDrive(m_left, m_right);
      //Left,Right PID controllers
      m_leftPIDController = new PIDController(DriveC.leftP, DriveC.leftI,DriveC.leftD);
      m_rightPIDController = new PIDController(DriveC.rightP, DriveC.rightI,DriveC.rightD);
      //Left, Right Encoders
      m_leftencoder = new Encoder(Ports.LeftDriveEncoder1, Ports.LeftDriveEncoder2,DriveC.leftEncoderReversed);
      m_rightencoder = new Encoder(Ports.RightDriveEncoder1, Ports.RightDriveEncoder2,DriveC.rightEncoderReversed);
      m_kinematics = new DifferentialDriveKinematics(DriveC.kTrackWidth_m);
    }catch (RuntimeException ex) {
      DriverStation.reportError("Error Configuring Drivetrain" + ex.getMessage(), true);
    }
    try{
      // Need to set drive characterization constants from SysID in Metric units
      m_feedforward = new SimpleMotorFeedforward(DriveC.ksVolts,DriveC.kvVoltSecondsPerMeter,DriveC.kaVoltSecondsSquaredPerMeter);
      m_leftencoder.setDistancePerPulse(DriveC.distancePerPulse_m);
      m_rightencoder.setDistancePerPulse(DriveC.distancePerPulse_m);
      m_rightencoder.setReverseDirection(true);
      SmartDashboard.putData("Field", m_field);
      m_powerdist = new PowerDistribution(1, ModuleType.kRev);
      SmartDashboard.putData("PDP", m_powerdist);
    }catch (RuntimeException ex) {
      DriverStation.reportError("Error Configuring Drivetrain" + ex.getMessage(), true);
    }
    addChild("Right Encoder", m_rightencoder);
    addChild("Left Encoder", m_leftencoder);
    addChild("Right PID", m_rightPIDController);
    addChild("Left PID", m_leftPIDController);
    try {
      ahrs = new AHRS(SerialPort.Port.kUSB1);
      ahrs.enableLogging(true);
      m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d());
      addChild("Gyro", ahrs);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }
    addChild("DriveTrain",m_drive);

  }

    public double getAverageDistance() {
      return (m_leftencoder.getDistance() + m_rightencoder.getDistance()) / 2;
    }

    public double getLeftDistance() {
      return (m_leftencoder.getDistance());
    }

    public double getRightDistance() {
      return (m_rightencoder.getDistance());
    }

    public void resetEncoders() {
      m_leftencoder.reset();
      m_rightencoder.reset();
    }

    public void setMaxOutput(double newMax) {
      m_drive.setMaxOutput(newMax);
      maxspeed = newMax;
    }

    public void toggleMaxSpeed() {
      if (maxspeed == DriveC.MaxSpeed) {
        setMaxOutput(DriveC.reducedMaxSpeed);
      } else {
        setMaxOutput(DriveC.MaxSpeed);
      }
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
      final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
      final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
      final double leftOutput  = m_leftPIDController.calculate(m_leftencoder.getRate(), speeds.leftMetersPerSecond);
      final double rightOutput = m_rightPIDController.calculate(m_rightencoder.getRate(), speeds.rightMetersPerSecond);
      m_left.setVoltage(leftOutput + leftFeedforward);
      m_right.setVoltage(rightOutput + rightFeedforward);
    }

    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rot) {
      var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
      setSpeeds(wheelSpeeds);
    }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
        ahrs.getRotation2d(), m_leftencoder.getDistance(), m_rightencoder.getDistance());
  }

  public void TankDrive(double leftspeed, double rightspeed) {
    final double leftFeedforward = m_feedforward.calculate(leftspeed);
    final double rightFeedforward = m_feedforward.calculate(rightspeed);
    final double leftOutput  = m_leftPIDController.calculate(m_leftencoder.getRate(), leftspeed);
    final double rightOutput = m_rightPIDController.calculate(m_rightencoder.getRate(), rightspeed);
    m_left.setVoltage(leftOutput + leftFeedforward);
    m_right.setVoltage(rightOutput + rightFeedforward);
    //m_drive.tankDrive(leftspeed, rightspeed);
  }

  public void ArcadeDrive(double forwardspeed, double rotationspeed) {
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
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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
    updateOdometry();
    SmartDashboard.putNumber("Encoder Distance", getAverageDistance());
    SmartDashboard.putNumber("Right Encoder Distance", getRightDistance());
    SmartDashboard.putNumber("Left Encoder Distance", getLeftDistance());
    SmartDashboard.putNumber("Gyro", getHeading());
    m_field.setRobotPose(m_odometry.getPoseMeters());
   }
}
