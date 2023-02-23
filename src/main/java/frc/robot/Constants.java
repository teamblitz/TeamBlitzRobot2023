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
import edu.wpi.first.math.geometry.Translation3d;
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
        public static final int CURRENT_LIMIT = 20;

        // Which intake is on the bot?
        public enum Type {
            Simple,
            Complex
        }

        public static final Type type = Type.Simple;

        public static final class Simple {
            public static final double IN_SPEED = 0.2;
            public static final double OUT_SPEED = -0.2;
            // From robot POV
            public static final int LEFT_MOTOR_ID = 13;
            public static final int RIGHT_MOTORI_ID = 14;
        }

        public static final class Complex {
            public static final double IN_SPEED = 0.2;
            public static final double OUT_SPEED = -0.2;

            public static final int FRONT_MOTOR_ID = 13;
            public static final int BACK_MOTOR_ID = 14;
        }
    }

    public static final class Arm {

        public static final int ARM_ROT_LEADER = 18; // Like the above IDs, may need to be adjusted
        public static final int ARM_ROT_FOLLOWER = 19;

        public static final int WRIST_ROT_LEADER = 20;
        public static final int WRIST_ROT_FOLLOWER = 21;

        public static final int ARM_EXTENSION_LEADER = 22;
        public static final int ARM_EXTENSION_FOLLOWER = 23;

        /** The center of the center of rotation for the arm */
        public static final Translation3d ARM_BASE =
                new Translation3d(0, 0, 0); // TODO: Tune to robot

        public static final Rotation2d LOWEST_ROTATION = new Rotation2d(); // TODO: Tune to robot
        public static final Rotation2d HIGHEST_ROTATION = new Rotation2d(); // TODO: Tune to robot

        /** End of arm distance from the center of rotation when the arm is not extended */
        public static final int MIN_EXTENSION = 1;
        /**
         * Distance of the end of the arm from the center of rotation when the arm is fully
         * extended.
         */
        public static final int MAX_EXTENSION = 2; // TODO: Tune to robot

        public static final double ROTATION_GEAR_RATIO = 1.0 / 16.0; // TODO: Tune to robot
        public static final double EXTENSION_GEAR_RATION = 0; // TODO: Tune to robot

        // Values to determine if a configuration of the arm is legal.
        public static final double MAX_LEGAL_HEIGHT = Units.feetToMeters(6.5);
        public static final double MAX_EXTENSION_PAST_Frame = Units.inchesToMeters(48);
    }

    public static final class Swerve {
        public static final int PIGEON_ID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

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
        public static final double ANGLE_KP = 0.004; // TODO: Tune this.
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;
        public static final double ANGLE_KF = 0.0; // For now, should remain zero

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.05; // TODO: This must be tuned to specific robot
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0; // Same here.

        /* Drive Motor Characterization Values in volts*/
        public static final double DRIVE_KS = (0.32); // TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = (1.51);
        public static final double DRIVE_KA = (0.27);

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
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(19.68);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 2;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(258.13);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CAN_CODER_ID = 13;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(235.28);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 9;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CAN_CODER_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(18.54);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(
                            DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                        MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class OIConstants {

        // public static final Function<Double, Double> inputCurve = (x) -> x;
        public static final Function<Double, Double> inputCurve = (x) -> .6 * x + .4 * (x * x * x);

        // Choose 1, not both.
        public static final boolean USE_XBOX_CONTROLLER = false;
        public static final boolean USE_SAITEK_CONTROLLER = true;

        public static final int DRIVE_CONTROLLER_PORT = 0;
        public static final int BUTTON_BOX_PORT = 1;

        public static final class XboxMappings {
            // We might not use xbox controller
        }

        public static final class SaitekMappings {
        }

        public static final class ButtonBoxMappings {
            public static final ButtonBox.Button UP_ARM = ButtonBox.Button.kA;
            public static final ButtonBox.Button DOWN_ARM = ButtonBox.Button.kB;

        }
    }

    // TODO: Calculate needed deadband for controller (should be like 6% or less)
    public static double STICK_DEADBAND = 0.04;
}
