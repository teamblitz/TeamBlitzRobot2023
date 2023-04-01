package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.commands.ManipulatorCommandFactory;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.Logger;

// This class is mainly just storage for the autonomous pathing and commands
public class AutonomousPathCommand {
    private final DriveSubsystem driveSubsystem;
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ManipulatorCommandFactory manipulatorCommandFactory;

    private final Logger logger = Logger.getInstance();

    private static final boolean backupAuto = false;

    public AutonomousPathCommand(
            final DriveSubsystem driveSubsystem,
            final ArmSubsystem armSubsystem,
            final IntakeSubsystem intakeSubsystem,
            final ManipulatorCommandFactory manipulatorCommandFactory) {
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.manipulatorCommandFactory = manipulatorCommandFactory;
    }

    // Intake Commands
    public Command autoConeIn() {
        return this.intakeSubsystem.buildConeInCommand().withTimeout(2);
    }

    public Command autoConeOut() {
        return this.intakeSubsystem.buildConeOutCommand().withTimeout(1);
    }

    public Command autoCubeIn() {
        return this.intakeSubsystem.buildCubeInCommand().withTimeout(2);
    }

    public Command autoCubeOut() {
        return this.intakeSubsystem.buildCubeOutCommand().withTimeout(1);
    }

    public Command driveOutDistance(double distance) {
        return Commands.run(
                        () -> driveSubsystem.drive(new Translation2d(1, 0), 0, true, true, false),
                        driveSubsystem)
                .until(
                        () ->
                                Math.abs(driveSubsystem.getPose().getX()) > distance
                                        || Math.abs(driveSubsystem.getPose().getY()) > distance)
                .finallyDo((b) -> driveSubsystem.drive(new Translation2d(), 0, false, true, false));
    }

    public Command driveBackDistance(double distance) {
        return Commands.run(
                        () -> driveSubsystem.drive(new Translation2d(-1, 0), 0, true, true, false),
                        driveSubsystem)
                // .until(() -> Math.abs(driveSubsystem.getPose().getX()) < distance ||
                // Math.abs(driveSubsystem.getPose().getY()) < distance)
                .finallyDo((b) -> driveSubsystem.drive(new Translation2d(), 0, false, true, false));
    }

    public Command autoMidCube() {
        return this.manipulatorCommandFactory
                .primeCubeMid()
                .withTimeout(2)
                .beforeStarting(() -> logger.recordOutput("auto/state", "prime"))
                .andThen(
                        this.intakeSubsystem
                                .buildCubeOutCommand()
                                .withTimeout(0.5)
                                .beforeStarting(() -> logger.recordOutput("auto/state", "cubeOut")))
                .andThen(
                        this.armSubsystem
                                .homeArmCommand()
                                .withTimeout(2)
                                // .andThen(Commands.run(() -> {}))
                                .beforeStarting(
                                        () -> logger.recordOutput("auto/state", "homeArm")));
    }

    public Command generateAutonomous(String path) {
        if (backupAuto) {
            return generateBackupAuto(path);
        }

        final List<PathPlannerTrajectory> pathGroup;
        HashMap<String, Command> eventMap = new HashMap<>();
        // This will load the file "FullAuto.path"
        // All paths are in /src/main/deploy/pathplanner
        // Please set robot width/length in PathPlanner to 36.5 x 36.5 inches- > meters (0.9271)

        // Check timeouts! Please test.
        // I'm still treating the prime commands like they do not end.
        // ----- Arm Positions -----
        eventMap.put("armHome", this.armSubsystem.homeArmCommand());
        eventMap.put(
                "armConeGround",
                this.manipulatorCommandFactory.groundUprightConePickup().withTimeout(3));
        eventMap.put("armConeHigh", this.manipulatorCommandFactory.primeConeHigh().withTimeout(3));
        eventMap.put(
                "armCubeGround", this.manipulatorCommandFactory.groundCubePickup().withTimeout(3));
        eventMap.put("armCubeMid", this.manipulatorCommandFactory.primeCubeMid().withTimeout(3));
        eventMap.put("armCubeHigh", this.manipulatorCommandFactory.primeCubeHigh().withTimeout(3));

        // ----- Intake -----
        eventMap.put("intakeCube", this.intakeSubsystem.buildCubeInCommand().withTimeout(1));
        eventMap.put("intakeCone", this.intakeSubsystem.buildConeInCommand().withTimeout(1));

        // ----- Outtake -----
        eventMap.put("outtake", this.intakeSubsystem.buildCubeOutCommand().withTimeout(0.25));

        // ----- Balance -----
        eventMap.put("balance", new AutoBalance(this.driveSubsystem));
        eventMap.put("autoCubeMid", this.autoCubeOut());
        eventMap.put("buildPark", this.driveSubsystem.buildParkCommand().withTimeout(3));

        // ----- Testing -----
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("marker2", new PrintCommand("Passed marker 2"));

        if (path == "Score") {
            return autoCubeOut();
        } else if (path == "BalanceTest") {
            return Commands.run(
                            () ->
                                    this.driveSubsystem.drive(
                                            new Translation2d(-.75, 0), 0, false, true, false))
                    .until(() -> Math.abs(this.driveSubsystem.getPitch()) > 12)
                    .andThen(() -> this.driveSubsystem.setBrakeMode(true))
                    .andThen(new AutoBalance(this.driveSubsystem))
                    .andThen(this.driveSubsystem.buildParkCommand().repeatedly());
        } else if (path == "Nothing") {
            return null;
        } else {
            pathGroup = PathPlanner.loadPathGroup(path, new PathConstraints(2, 1.5));
        }

        // Create the AutoBuilder
        SwerveAutoBuilder autoBuilder =
                new SwerveAutoBuilder(
                        // Pose2d supplier
                        this.driveSubsystem::getPose,
                        // Pose2d consumer (should be resetPose, fix later)
                        this.driveSubsystem::resetOdometry,
                        // SwerveDriveKinematics
                        Constants.Swerve.KINEMATICS,
                        // Use DrivePID presumably
                        new PIDConstants(
                                Constants.Swerve.DRIVE_KP,
                                Constants.Swerve.DRIVE_KI,
                                Constants.Swerve.DRIVE_KD),
                        // Use AnglePID presumably
                        new PIDConstants(
                                Constants.Swerve.ANGLE_KP,
                                Constants.Swerve.ANGLE_KI,
                                Constants.Swerve.ANGLE_KD),
                        // Module states consumer used to output the drive subsystem
                        (states) ->
                                this.driveSubsystem.setModuleStates(states, false, false, false),
                        eventMap,
                        // Should the path be automatically mirrored depending on alliance color
                        true,
                        this.driveSubsystem);
        return autoBuilder.fullAuto(pathGroup);
    }

    public Command generateBackupAuto(String path) {
        switch (path) {
            case "Score":
                // return autoMidCube();
                // .andThen(() -> System.out.println("Score"))
                // .andThen(driveSubsystem.buildParkCommand().repeatedly());
                return this.autoCubeOut();
            case "Left":
                return autoMidCube()
                        .andThen(() -> System.out.println("Left pre drive"))
                        .andThen(
                                driveOutDistance(2.5).withTimeout(2.5),
                                driveBackDistance(2).withTimeout(2))
                        .andThen(driveSubsystem.buildParkCommand().repeatedly());
            case "Right":
                return autoMidCube()
                        .andThen(() -> System.out.println("Right pre drive"))
                        .andThen(
                                driveOutDistance(4.5).withTimeout(4.5),
                                driveBackDistance(3).withTimeout(3))
                        .andThen(driveSubsystem.buildParkCommand().repeatedly());
            case "Middle":
                return autoMidCube()
                        .andThen(driveOutDistance(6).withTimeout(6))
                        .andThen(
                                Commands.run(
                                                () ->
                                                        driveSubsystem.drive(
                                                                new Translation2d(-1, 0),
                                                                0,
                                                                true,
                                                                true,
                                                                false),
                                                driveSubsystem)
                                        .until(() -> Math.abs(driveSubsystem.getPitch()) < -10)
                                        .finallyDo(
                                                (b) ->
                                                        driveSubsystem.drive(
                                                                new Translation2d(0, 0),
                                                                0,
                                                                true,
                                                                true,
                                                                false))
                                        .andThen(driveSubsystem.buildParkCommand().repeatedly()));
            case "Balance":
                // return autoMidCube()
                // .andThen(
                //         driveOutDistance(4).withTimeout(4)
                // ).andThen(
                //         Commands.run(
                //                 () -> driveSubsystem.drive(new Translation2d(-1, 0), 0, true,
                // true, false), driveSubsystem
                //         ).until(() -> Math.abs(driveSubsystem.getPitch()) < -10)
                //         .andThen(() -> driveSubsystem.drive(new Translation2d(speedCalculation(),
                // 0), 0, false, true, false))
                // );

                return autoMidCube()
                        .andThen(
                                Commands.run(
                                                () ->
                                                        driveSubsystem.drive(
                                                                new Translation2d(1, 0),
                                                                0,
                                                                true,
                                                                true,
                                                                false),
                                                driveSubsystem)
                                        .withTimeout(6))
                        .until(() -> driveSubsystem.getPitch() > 15)
                        .andThen(
                                Commands.run(
                                                () ->
                                                        driveSubsystem.drive(
                                                                new Translation2d(.75, 0),
                                                                0,
                                                                true,
                                                                true,
                                                                false),
                                                driveSubsystem)
                                        .withTimeout(4))
                        .until(() -> driveSubsystem.getPitch() < 10)
                        .finallyDo(
                                (b) ->
                                        driveSubsystem.drive(
                                                new Translation2d(0, 0), 0, false, true, false))
                        .andThen(driveSubsystem.buildParkCommand().repeatedly());
            default:
                return autoMidCube();
        }
    }
}
