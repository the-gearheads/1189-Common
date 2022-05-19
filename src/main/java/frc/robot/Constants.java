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
        public static final double SHAFT_TO_WHEEL_GEAR_RATIO = 0;
        public static final double WHEEL_CIRCUMFERENCE = 0;
        public static final double TRACK_WIDTH = 0;
        public static final Pose2d START_POSITION = new Pose2d(0,0, new Rotation2d(0));
        public static final double LEFT_FF_kS = 0;
        public static final double LEFT_FF_kV = 0;
        public static final double RIGHT_FF_kS = 0;
        public static final double RIGHT_FF_kV = 0;
    }
}
