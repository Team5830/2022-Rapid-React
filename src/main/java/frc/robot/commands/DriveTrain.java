// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import  frc.robot.Constants;
/** An example command that uses an example subsystem. */
public class DriveTrain extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  WPI_VictorSPX m_rightlead = new WPI_VictorSPX(Constants.CANBusID.rightkMotor1);
  WPI_VictorSPX m_rightfollow = new WPI_VictorSPX(Constants.CANBusID.rightkMotor2);
  WPI_VictorSPX m_leftlead = new WPI_VictorSPX(Constants.CANBusID.leftkMotor1);
  WPI_VictorSPX m_leftfollow = new WPI_VictorSPX(Constants.CANBusID.leftkMotor1);
  DifferentialDrive m_drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveTrain() {
    //m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
    m_rightlead.setInverted(true);
    m_rightfollow.setInverted(true);
    m_rightfollow.follow(m_rightlead);
    m_leftfollow.follow(m_leftlead);
    m_drive = new DifferentialDrive(m_leftlead, m_rightlead);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public void TankDrive(double leftspeed, double rightspeed){
    m_drive.tankDrive(leftspeed, rightspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
