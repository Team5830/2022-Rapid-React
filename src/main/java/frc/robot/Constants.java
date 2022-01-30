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
        public static final int kLeftMotor1 = 4;
        public static final int kRightMotor1 = 11;
        public static final int kLeftMotor2 = 7;
        public static final int kRightMotor2 = 12;
    }
    public static final class Ports {
        public static final int LeftDriveEncoder1 = 0;
        public static final int LeftDriveEncoder2 = 1;
        public static final int RightDriveEncoder1 = 2;
        public static final int RightDriveEncoder2 = 3;

    }
    public static final class Drive {    
        public static final double reducedMaxSpeed =0.5;
        public static final double MaxSpeed =1.0;
        public static final boolean SquareInputs = true;
        public static final int kEncoderCPR = 20;
        public static final double kWheelCircInches = 18.5;
        public static final double distancePerPulse = (kWheelCircInches) / (double) kEncoderCPR/10.71;//Gear Ratio = 1:10.71
    }
    public static final class buttonsLeftjoy {
        public static final int halfspeedButton =5;
    }
}
