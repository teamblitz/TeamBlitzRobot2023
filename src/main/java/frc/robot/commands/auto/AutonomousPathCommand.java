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
import frc.robot.commands.CommandBuilder;
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
    private final CommandBuilder commandBuilder;

    private final Logger logger = Logger.getInstance();

    private static final boolean backupAuto = false;

    public AutonomousPathCommand(
            final DriveSubsystem driveSubsystem,
            final ArmSubsystem armSubsystem,
            final IntakeSubsystem intakeSubsystem,
            final CommandBuilder commandBuilder) {
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.commandBuilder = commandBuilder;
    }

    // Add vision at some point. To this one specifically, but the whole autonomous would be nice.
    public Command balanceChargeStation() {
        return Commands.run(
                        () ->
                                this.driveSubsystem.drive(
                                        new Translation2d(speedCalculation(), 0),
                                        0,
                                        false,
                                        true,
                                        false),
                        driveSubsystem)
                .until(() -> this.driveSubsystem.getPitch() <= 10);
    }

    private double speedCalculation() {
        if (this.driveSubsystem.getPitch() > 10) {
            return -0.2;
        } else if (this.driveSubsystem.getPitch() <= -10) {
            return 0.2;
        } else {
            return 0;
        }
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

    // Mid Cube command (used in all autonomous)
    // This does not stop in simulation because encoders don't get values...
    public Command autoMidCube() {
        return this.commandBuilder
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
        // Please set robot width/length in PathPlanner to 34 x 34 inches --> meters (0.8636 meters)

        // eventMap.put("autoCubeMid", this.autoMidCube());
        eventMap.put(
                "autoCubeMid",
                this.autoCubeOut()); // Changed to autoCubeOut to only drop cube, not move arm
        eventMap.put("balanceChargeStation", new AutoBalance(driveSubsystem, null)); // import fix later
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("marker2", new PrintCommand("Passed marker 2"));

        switch (path) {
            case "Left":
                pathGroup = PathPlanner.loadPathGroup("Left", new PathConstraints(2, 1.5));
                break;
            case "Middle":
                pathGroup = PathPlanner.loadPathGroup("Middle", new PathConstraints(2, 1.5));
                break;
            case "Right":
                pathGroup = PathPlanner.loadPathGroup("Right", new PathConstraints(2, 1.5));
                break;
            case "Nothing":
                return null;
                // pathGroup = PathPlanner.loadPathGroup("Nothing", new PathConstraints(2, 1.5));
                // break;
            case "Test":
                pathGroup = PathPlanner.loadPathGroup("SquarePath", new PathConstraints(2, 1.5));
                break;
            default:
                pathGroup = PathPlanner.loadPathGroup("SquarePath", new PathConstraints(2, 1.5));
                break;
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
