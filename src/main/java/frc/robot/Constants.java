// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CANBusID{
        public static final int leftkMotor1 = 4;
        public static final int rightkMotor1 = 11;
        public static final int leftkMotor2 = 7;
        public static final int rightkMotor2 = 12;
    }
    public static final class drive{
        public static final double MaxSpeed = 1.0;
        public static final double reducedMaxSpeed = 0.5; 
    }
    public static final class buttonsLeftjoy{
        public static final int halfspeedButton = 1;
    }
    public static final class buttonsRightjoy{
        public static final int button1 = 1;
    }

}
