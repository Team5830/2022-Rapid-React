// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SmartDashboardCmd extends CommandBase {
  private final DriveTrain m_subsystemDriveTrain;

  /** Creates a new SmartDashboard. */
  public SmartDashboardCmd(DriveTrain m_subsystemDriveTrain) {
    this.m_subsystemDriveTrain = m_subsystemDriveTrain;
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
