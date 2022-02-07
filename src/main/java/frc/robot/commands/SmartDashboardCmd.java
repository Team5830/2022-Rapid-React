// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;

public class SmartDashboardCmd extends CommandBase {
  private final DriveTrain m_subsystemDriveTrain;
  private final Flywheel m_subsystemFlywheel;

  /** Creates a new SmartDashboard. */
  public SmartDashboardCmd(DriveTrain m_subsystemDriveTrain, Flywheel m_subsystemFlywheel) {
    this.m_subsystemDriveTrain = m_subsystemDriveTrain;
    this.m_subsystemFlywheel = m_subsystemFlywheel;

    SmartDashboard.putNumber("P Gain", this.m_subsystemFlywheel.pidVals.kP);
    SmartDashboard.putNumber("I Gain", this.m_subsystemFlywheel.pidVals.kI);
    SmartDashboard.putNumber("D Gain", this.m_subsystemFlywheel.pidVals.kD);
    SmartDashboard.putNumber("I Zone", this.m_subsystemFlywheel.pidVals.kIz);
    SmartDashboard.putNumber("Feed Forward", this.m_subsystemFlywheel.pidVals.kFF);
    SmartDashboard.putNumber("Max Output", this.m_subsystemFlywheel.pidVals.kMaxOutput);
    SmartDashboard.putNumber("Min Output", this.m_subsystemFlywheel.pidVals.kMinOutput);
    SmartDashboard.putNumber("Shooter Velocity", this.m_subsystemFlywheel.pidVals.motorspeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Encoder Distance", m_subsystemDriveTrain.getAverageDistance());
    SmartDashboard.putNumber("Right Encoder Distance", m_subsystemDriveTrain.getRightDistance());
    SmartDashboard.putNumber("Left Encoder Distance", m_subsystemDriveTrain.getLeftDistance());
    SmartDashboard.putNumber("Gyro Angle", m_subsystemDriveTrain.getHeading());

    this.m_subsystemFlywheel.pidVals.kP = SmartDashboard.getNumber("P Gain", 0);
    this.m_subsystemFlywheel.pidVals.kI = SmartDashboard.getNumber("I Gain", 0);
    this.m_subsystemFlywheel.pidVals.kD = SmartDashboard.getNumber("D Gain", 0);
    this.m_subsystemFlywheel.pidVals.kIz = SmartDashboard.getNumber("I Zone", 0);
    this.m_subsystemFlywheel.pidVals.kFF = SmartDashboard.getNumber("Feed Forward", 0);
    this.m_subsystemFlywheel.pidVals.kMaxOutput = SmartDashboard.getNumber("Max Output", 0);
    this.m_subsystemFlywheel.pidVals.kMinOutput = SmartDashboard.getNumber("Min Output", 0);
    this.m_subsystemFlywheel.pidVals.motorspeed = SmartDashboard.getNumber("Shooter Velocity", 0);
    SmartDashboard.putBoolean("Shooter On", this.m_subsystemFlywheel.getShooterState());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
