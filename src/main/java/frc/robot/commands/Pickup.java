// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.FirstIntake;
import frc.robot.subsystems.Flywheel;

public class Pickup extends SequentialCommandGroup {
  public Pickup(FirstIntake m_intake, Conveyor m_conveyor, Flywheel m_flywheel) {
    addRequirements(m_intake, m_conveyor, m_flywheel);
    addCommands(
        new ParallelCommandGroup(
            new InstantCommand(m_intake::intakeDown),
            new InstantCommand(m_intake::startFirstIntake),
            new InstantCommand(m_conveyor::conveyor1ON),
            new InstantCommand(m_conveyor::conveyor2ON)),
        new Pause(0.5),
        new WaitUntilCommand(() -> m_conveyor.jammed),
        new InstantCommand(m_conveyor::conveyor2OFF),
        new InstantCommand(m_conveyor::conv2down)
    // Pnew InstantCommand(m_flywheel::shooteron)
    );
    /*
     * addCommands(
     * new ParallelCommandGroup(
     * new InstantCommand(m_intake::startFirstIntake),
     * new InstantCommand(m_conveyor::conveyor1ON)));
     */
  }
}
