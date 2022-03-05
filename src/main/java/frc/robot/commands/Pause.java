package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Timer;

public class Pause extends InstantCommand {
    public Pause() {
        super();
    }

    @Override
    public void initialize() {
        new Thread() {
            public void run() {
                Timer.delay(1);
            }
        }.start();
    }
}