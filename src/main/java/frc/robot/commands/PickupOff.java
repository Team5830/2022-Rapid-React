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

public class PickupOff extends SequentialCommandGroup {
    public PickupOff(FirstIntake m_intake, Conveyor m_conveyor) {
        addRequirements(m_intake, m_conveyor);
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(m_intake::stopFirstIntake),
                        new InstantCommand(m_conveyor::conveyor1OFF)));
    }
}
