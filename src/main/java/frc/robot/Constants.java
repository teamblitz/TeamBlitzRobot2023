/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.oi.ButtonBox;
import frc.lib.util.COTSSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import java.util.Arrays;
import java.util.List;
import java.util.function.Function;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final Mode simMode = Mode.SIM;

    public static final boolean tuningMode = true;

    public enum Mode {
        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class Intake {

        /*
         * Neo 550
         * 20A Limit - Motor survived full 220s test.
         * 40A Limit - Motor failure at approximately 27s.
         * 60A Limit - Motor failure at approximately 5.5s
         * 80A Limit* - Motor failure at approximately 2.0s
         *
         * For NEOs use 40
         * https://www.revrobotics.com/neo-brushless-motor-locked-rotor-testing/
         * */
        public static final int CURRENT_LIMIT = 40;

        // Which intake is on the bot?
        public enum Type {
            Simple,
            Complex
        }

        public static final Type type = Type.Simple;

        public static final class Simple {
            public static final double IN_SPEED = 0.5;
            public static final double OUT_SPEED = -0.5;
            // From robot POV
            public static final int LEFT_MOTOR_ID = 14;
            public static final int RIGHT_MOTOR_ID = 15;
        }

        public static final class Complex {
            public static final double IN_SPEED = 0.2;
            public static final double OUT_SPEED = -0.2;

            public static final int FRONT_MOTOR_ID = 14;
            public static final int BACK_MOTOR_ID = 15;

            
        }
        public static final int DETECTION_CURRENT_THRESHOLD = 0; //needs value

    }

    public static final class Arm {

        public static final int ARM_ROT_LEADER = 18; // Right
        public static final int ARM_ROT_FOLLOWER = 19; // Left

        public static final int WRIST_ROT_LEADER = 20;

        public static final int ARM_EXTENSION_LEADER = 17; // Left
        public static final int ARM_EXTENSION_FOLLOWER = 16; // Right

        public static final int TOP_ROTATION_LIMIT_SWITCH = 0;
        public static final int BOTTOM_ROTATION_LIMIT_SWITCH = 1;

        public static final int TOP_EXTENSION_LIMIT_SWITCH = 4;
        public static final int BOTTOM_EXTENSION_LIMIT_SWITCH = 5;

        public static final int TOP_WRIST_LIMIT_SWITCH = 2;
        public static final int BOTTOM_WRIST_LIMIT_SWITCH = 3;

        public static final int ABS_ROTATION_ENCODER = 6;
        public static final int ABS_WRIST_ENCODER = 7;

        public static final double ARM_ROT_OFFSET = -104 - 90;

        public static final double STARTING_ROTATION = 90;
        /** The center of the center of rotation for the arm */
        public static final double ARM_BASE_HEIGHT =
                Units.inchesToMeters(19); // TODO: Tune to robot

        public static final double ARM_BASE_DISTANCE_FROM_FRAME = Units.inchesToMeters(14);

        public static final double MIN_ROT = -25; // TODO: Tune to robot
        public static final double MAX_ROT = 90; // TODO: Tune to robot

        public static final double ROT_THRESHOLD = 10;

        public static final double EXTENSION_THRESHOLD =
                Units.inchesToMeters(1); // Tight, but achievable.
        public static final double MIN_EXTENSION = 0;

        public static final double MAX_EXTENSION = 1.10; // TODO: Tune to robot

        public static final double IN_LENGTH = Units.inchesToMeters(32);
        public static final double OUT_LENGTH = Units.inchesToMeters(60);

        public static final double HEIGHT_PROTECTION = Units.feetToMeters(5.5);
        public static final double PULL_TO = .3;

        public static final double TUCK_IN_EXTENSION = Units.inchesToMeters(25); // TODO: TUNE

        public static final double SLOW_DOWN_AT = Units.inchesToMeters(30);

        public static final double STOP_EXTENSION = Units.inchesToMeters(41);

        public static final double ROTATION_GEAR_RATIO = (60.0 / 1.0);
        public static final double EXTENSION_GEAR_RATIO = (48.0 / 1.0); // TODO: Confirm this

        public static final double WRIST_GEAR_RATIO = (27.0 / 1.0); // TODO: Get the actual value
        public static final double
                EXTENSION_PULLEY_CIRCUMFERENCE = // This should be good enough for now
                0.0191 * Math.PI; // diameter in meters

        // Values to determine if a configuration of the arm is legal.
        public static final double MAX_LEGAL_HEIGHT = Units.feetToMeters(6.5);
        public static final double MAX_EXTENSION_PAST_FRAME = Units.inchesToMeters(48);

        // Units in degrees per second and degrees per second squared
        public static final double ROTATION_VELOCITY = 30;
        public static final double ROTATION_ACCELERATION = 60; // 1 seconds to full

        // Units in meters per second and meters per second squared
        public static final double EXTENSION_VELOCITY = 1;
        public static final double EXTENSION_ACCELERATION =
                EXTENSION_VELOCITY / .5; // .5 seconds from 0 to full

        public static final double ROT_P = .07;
        public static final double ROT_I = 0;
        public static final double ROT_D = 0.01;

        public static final double EXT_P = .007;
        public static final double EXT_I = 0;
        public static final double EXT_D = 0;

        public static final class Position {

            public static final class Extension {
                public static final double RETRACTED = .05;

                public static final double CONE_HIGH = .76; // or .73
                public static final double CONE_MID = .23;
                public static final double CUBE_HIGH = .4; // TODO: TUNE
                public static final double CUBE_MID = 0;

                public static final double HYBRID = 0;

                public static final double CUBE_PICKUP_GROUND = 0;
                public static final double CONE_UPRIGHT_PICKUP_GROUND = 0;
                public static final double CONE_FALLEN_PICKUP_GROUND = 0;

                public static final double CUBE_RAMP = 0;
                public static final double CONE_RAMP = 0;
                public static final double CUBE_SHELF = 1; // TODO: TUNE
                public static final double CONE_SHELF = 1; //  TODO: TUNE
            }

            public static final class Rotation {
                public static final double LEVEL = 0;
                public static final double VERTICAL = 90;

                public static final double CONE_HIGH = 31;
                public static final double CONE_MID = 34;
                public static final double CUBE_HIGH = 23.7;
                public static final double CUBE_MID = 17.5;

                public static final double CUBE_PICKUP_GROUND = -20;
                public static final double CONE_UPRIGHT_PICKUP_GROUND = -20;
                public static final double CONE_FALLEN_PICKUP_GROUND = -20;

                public static final double HYBRID = -10; // TODO: TUNE

                public static final double CUBE_RAMP = 30; // TODO: TUNE
                public static final double CONE_RAMP = 30; // TODO: TUNE

                public static final double CUBE_SHELF = 50; // TODO: TUNE

                public static final double CONE_SHELF = 50; //  TODO: TUNE
            }
        }
    }

    public static final class Wrist {

        public static final double MIN_ROTATION = -177; // TODO: TUNE THESE.
        public static final double MAX_ROTATION = -23;
        public static final double ks = 0.41856;
        public static final double kg = 0.89932;
        public static final double kv = 0.24084;
        // ka = 0.013184
        // All in volts, divide by 12 to percent

        public static final double p = 0.008;
        public static final double i = 0;
        public static final double d = 0.0001;

        public static final double ROTATION_VELOCITY = 60;
        public static final double ROTATION_ACCELERATION = 120; // 1 seconds to full

        public static final double ENCODER_OFFSET = -165 + 90;
        //        public static final double CG_OFFSET = Math.toDegrees(1.1423);
        public static final double CG_OFFSET = 0;

        public static final class Position {
            public static final double STARTING = -175;
            public static final double TUCKED_IN = -170;

            public static final double LEVEL = 0;

            public static final double VERTICAL = -90;

            public static final double CONE_HIGH_RELATIVE = -50;
            public static final double CONE_MID_RELATIVE = -52;
            public static final double CUBE_HIGH_RELATIVE = 0;
            public static final double CUBE_MID_RELATIVE = 0;

            public static final double HYBRID_ROBOT_RELATIVE = 0;

            public static final double CUBE_PICKUP_GROUND_ROBOT_RELATIVE = 0;
            public static final double CONE_UPRIGHT_PICKUP_GROUND_ROBOT_RELATIVE = -20;
            public static final double CONE_FALLEN_PICKUP_GROUND_ROBOT_RELATIVE = -10;

            public static final double CUBE_RAMP_ROBOT_RELATIVE = 20; // TODO: TUNE
            public static final double CONE_RAMP_ROBOT_RELATIVE = 20; // TODO: TUNE

            public static final double CONE_SHELF_RELATIVE = -90;
            public static final double CUBE_SHELF_RELATIVE = 0;
        }
    }

    public static final class Swerve {
        public static final int PIGEON_ID = 30;
        public static final boolean USE_PIGEON = true;

        public static final COTSSwerveConstants chosenModule =
                COTSSwerveConstants.SDSMK4i(COTSSwerveConstants.driveGearRatios.SDSMK4i_L3);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH =
                Units.inchesToMeters(24.75); // TODO: This must be tuned to specific robot
        public static final double WHEEL_BASE =
                Units.inchesToMeters(24.75); // TODO: This must be tuned to specific robot
        public static final double WHEEL_CIRCUMFERENCE = chosenModule.wheelCircumference;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = chosenModule.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = chosenModule.driveMotorInvert;

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = chosenModule.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = chosenModule.angleGearRatio;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERT = chosenModule.canCoderInvert;

        public static final int FL = 0; // Front Left Module Index
        public static final int FR = 1; // Front Right Module Index
        public static final int BL = 2; // Back Left Module Index
        public static final int BR = 3; // Back Right Module Index

        public static final List<Translation2d> CENTER_TO_MODULE =
                Arrays.asList(
                        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        public static final SwerveDriveKinematics KINEMATICS =
                new SwerveDriveKinematics(
                        CENTER_TO_MODULE.get(FL),
                        CENTER_TO_MODULE.get(FR),
                        CENTER_TO_MODULE.get(BL),
                        CENTER_TO_MODULE.get(BR));

        /* Current Limits
         *
         * Current Limits attempt to prevent the motor from burning out under a stall condition, and prevent the breaker from being tripped.
         *
         * The current limits are from the controller to the motor, and aren't necessarily the same as the current coming from the battery.
         *
         * The smart current limit scales the current limit based on the speed of the motor (to make it better for closed loop control iirc),
         * Secondary limit turns off the motor until the current falls below the limit
         *
         * Not even sure if it does anything because the fuses are 40a
         */
        public static final int DRIVE_SMART_CURRENT_LIMIT = 40;
        public static final int DRIVE_SECONDARY_CURRENT_LIMIT = 65;

        public static final int ANGLE_SMART_CURRENT_LIMIT = 25;
        public static final int ANGLE_SECONDARY_CURRENT_LIMIT = 40;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 0.003; // TODO: Tune this.
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;
        public static final double ANGLE_KF = 0.0; // For now, should remain zero

        /* Drive Motor PID Values */
        /*
         * Some posible gains for kp
         * 0.0012347 60s denominator, don't convert
         * 0.074084 1s denominator, don't convert
         * 0.00063245 60s denominator, do convert 6.12 : 1
         */
        public static final double DRIVE_KP =
                0.0012347; // TODO: This must be tuned to specific robot
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0; // Same here.

        /* Drive Motor Characterization Values in volts*/
        public static final double DRIVE_KS = (0.19983);
        public static final double DRIVE_KV = (2.3923);
        public static final double DRIVE_KA = (0.47987);

        /* DriveSubsystem Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.6; // TODO: This must be tuned to specific robot
        /**
         * Radians per Second
         *
         * <p>Can likely be figured out using an equation. Or we can just tornado spin it and see
         * what happens.
         */
        public static final double MAX_ANGULAR_VELOCITY =
                10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode ANGLE_NEUTRAL_MODE = IdleMode.kCoast;
        public static final IdleMode DRIVE_NEUTRAL_MODE = IdleMode.kCoast;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 7;
            public static final int ANGLE_MOTOR_ID = 8;
            public static final int CAN_CODER_ID = 4;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(235.28 + 90);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 2;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(19.68 + 90);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CAN_CODER_ID = 13;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(18.54 + 90);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 9;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CAN_CODER_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(258.13 + 90);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants {

        public static final double CHARGE_STATION_MIN_ANGLE = 2;
        public static final double CHARGE_STATION_MAX_ANGLE = 10;

        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double PX_CONTROLLER = 1;
        public static final double PY_CONTROLLER = 1;
        public static final double P_THETA_CONTROLLER = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class OIConstants {

        // public static final Function<Double, Double> inputCurve = (x) -> x;
        public static final Function<Double, Double> inputCurve = (x) -> .8 * x + .2 * (x * x * x);

        // Choose 1, not both.
        public static final boolean USE_XBOX_CONTROLLER = false;
        public static final boolean USE_SAITEK_CONTROLLER = true;

        public static final int DRIVE_CONTROLLER_PORT = 0;
        public static final int BUTTON_BOX_PORT = 1;

        public static final class XboxMappings {
            // We might not use xbox controller
        }

        public static final class SaitekMappings {}

        public static final class ButtonBoxMappings {
            public static final ButtonBox.Button UP_ARM = ButtonBox.Button.kL3;
            public static final ButtonBox.Button DOWN_ARM = ButtonBox.Button.kR3;

            public static final ButtonBox.Button INTAKE_IN = ButtonBox.Button.kA;
            public static final ButtonBox.Button INTAKE_OUT = ButtonBox.Button.kB;

            public static final ButtonBox.Button ARM_IN = ButtonBox.Button.kX;

            public static final ButtonBox.Button ARM_OUT = ButtonBox.Button.kY;

            //            public static final Trigger intake = x

        }
    }

    // TODO: Calculate needed deadb7and for controller (should be like 6% or less)
    // Ran this in the pit; had issues with 10% upped to 12%
    public static double STICK_DEADBAND = 0.12;

    public static final class Networking {
        public static final String JETSON_IP_ADDRESS = "10.20.83.130";
        public static final int PORT = 5810;
        public static final int INTERVAL = 5;
    }
}
