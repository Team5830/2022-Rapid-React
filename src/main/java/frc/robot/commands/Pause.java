package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Timer;

public class Pause extends InstantCommand {
    public double dtime;

    public Pause(double delaytime) {
        super();
        dtime = delaytime;
    }

    @Override
    public void initialize() {
        Timer.delay(dtime);
    }
}
