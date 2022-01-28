// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Drive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_subsystem;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;
  
  public Drive(DriveTrain subsystemDT, DoubleSupplier left, DoubleSupplier right) {
    m_subsystem = subsystemDT;
    m_left = left;
    m_right = right;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {m_subsystem.TankDrive(m_left.getAsDouble(), m_right.getAsDouble());}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.TankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}