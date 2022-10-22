package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.FirstIntake;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Climber;

public class Alloff extends SequentialCommandGroup {
    public Alloff(FirstIntake m_intake, Conveyor m_conveyor, Flywheel m_flywheel, Climber m_climber) {
        addRequirements(m_intake, m_conveyor, m_flywheel);
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(m_intake::stopIntake),
                        new InstantCommand(m_intake::stopFirstIntake),
                        new InstantCommand(m_conveyor::conveyor1OFF),
                        new InstantCommand(m_conveyor::conveyor2OFF),
                        new InstantCommand(m_flywheel::shooteroff)));
    }
}
