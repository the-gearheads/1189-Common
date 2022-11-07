// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DRIVE{

        public static final int RFMOTOR_ID = 4;
        public static final int RBMOTOR_ID = 5;
        public static final int LFMOTOR_ID = 6;
        public static final int LBMOTOR_ID = 7;
        public static final int TALON_UNITS_PER_ROTATION = 2048;
        public static final double SHAFT_TO_WHEEL_GEAR_RATIO = 12.75;//12.75 rotations for encoder to 1 rotation for wheel
        public static final double WHEEL_CIRCUMFERENCE = 0.64;//meters
        public static final double TRACK_WIDTH = 0.68863;//meter
        // public static final Pose2d START_POSITION = new Pose2d(7.4,1.5, new Rotation2d(Math.PI/2));
        public static final Pose2d START_POSITION = new Pose2d(Units.inchesToMeters(55),Units.inchesToMeters(78), new Rotation2d(Math.PI));
        public static final double LEFT_FF_kS = 0.64993;
        public static final double LEFT_FF_kV = 2.0752;
        public static final double LEFT_FF_kA = 0.47984;
        public static final double RIGHT_FF_kS = 0.64184;
        public static final double RIGHT_FF_kV = 2.0963;
        public static final double RIGHT_FF_kA = 0.20007;
        public static final double COMBINED_FF_kS = 0.60156;
        public static final double COMBINED_FF_kV = 2.1364;
        public static final Pose2d ZERO_POSITION = new Pose2d(0,0, new Rotation2d(0));
        public static final double WHEEL_RADIUS = 0.1016;
        public static final double MAX_X_VEL = 2;
        public static final double MAX_ROT_VEL = 1;
        public static final Matrix<N5, N1> STATE_SD = new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02);
        public static final Matrix<N3, N1> LOCAL_SD = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01);
        public static final Matrix<N3, N1> GLOBAL_SD = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0001, 0.00001, 0.000001);      



        public static class SIM {
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

    public static class VISION{
        public static final HashMap<Integer, Pose3d> aprilTagPositions = new HashMap<Integer, Pose3d>() {{
            put(0,new Pose3d(Units.inchesToMeters(144),Units.inchesToMeters(31.5),Units.inchesToMeters(60), new Rotation3d(0,0,0)));
            put(1,new Pose3d(1,1,1, new Rotation3d(0,0,0)));
            put(2,new Pose3d(1,1,1, new Rotation3d(0,0,0)));
          }};
          
        public static final Transform3d cameraToRobot = new Transform3d(new Translation3d(Units.inchesToMeters(1),
                                                                                          Units.inchesToMeters(19),
                                                                                          Units.inchesToMeters(-73)),
                                                                                          new Rotation3d(0,0,Math.PI));
    }

    public static final class Controller{
        public static final double DRIVE_DEADBAND = 0.07;
        public static final double ROTATE_DEADBAND = 0.07;
    }
}
