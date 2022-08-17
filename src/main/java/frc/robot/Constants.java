// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveTrain{

        public static final int RFMOTOR_ID = 4;
        public static final int RBMOTOR_ID = 5;
        public static final int LFMOTOR_ID = 6;
        public static final int LBMOTOR_ID = 7;
        public static final int TALON_UNITS_PER_ROTATION = 2048;
        public static final double SHAFT_TO_WHEEL_GEAR_RATIO = 12.75;//12.75 rotations for encoder to 1 rotation for wheel
        public static final double WHEEL_CIRCUMFERENCE = 0.64;//meters
        public static final double TRACK_WIDTH = 1;//meter
        public static final Pose2d START_POSITION = new Pose2d(1,1, new Rotation2d(0));
        public static final double LEFT_FF_kS = 0;
        public static final double LEFT_FF_kV = 0;
        public static final double RIGHT_FF_kS = 0;
        public static final double RIGHT_FF_kV = 0;
        public static final Pose2d ZERO_POSITION = new Pose2d(0,0, new Rotation2d(0));
        public static final double WHEEL_RADIUS = 0;
        public static double kS;
        public static double kV;


        public static class Sim {
            // 100% incorrect, need to find out in cad
            public static final double JKG_M2 = 5;
            // in kg
            public static final double ROBOT_MASS = 56;

            public static final double LINEAR_KV = 1.9817;
            public static final double LINEAR_KA = 0.3182;

            public static final double ANGULAR_KV = 1.79;
            public static final double ANGULAR_KA = 0.69482;
        }
    }

    public static final class Controller{

        public static final double DRIVE_DEADBAND = 0.07;
        public static final double ROTATE_DEADBAND = 0.07;
    }
}
