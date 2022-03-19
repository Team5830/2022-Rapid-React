package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Timer;

public class Pause extends InstantCommand {
    public double delaySeconds;
    public Pause(double delay) {
        super();
        delaySeconds = delay;
    }

    @Override
    public void initialize() {
        Timer.delay(delaySeconds);
    }
}
