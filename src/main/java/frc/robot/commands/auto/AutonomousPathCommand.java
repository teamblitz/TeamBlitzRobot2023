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

// This class is mainly just storage for the autonomous pathing and commands
public class AutonomousPathCommand {
    private final DriveSubsystem driveSubsystem;
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final CommandBuilder commandBuilder;

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
            return 0.2;
        } else if (this.driveSubsystem.getPitch() <= -10) {
            return -0.2;
        } else {
            return 0;
        }
    }

    // Intake Commands
    // ----- TODO: Test timeout -----
    public Command autoConeIn() {
        return this.intakeSubsystem.buildConeInCommand().withTimeout(2);
    }

    public Command autoConeOut() {
        return this.intakeSubsystem.buildConeOutCommand().withTimeout(0.50);
    }

    public Command autoCubeIn() {
        return this.intakeSubsystem.buildCubeInCommand().withTimeout(2);
    }

    public Command autoCubeOut() {
        return this.intakeSubsystem.buildCubeOutCommand().withTimeout(0.50);
    }

    // Mid Cube command (used in all autonomous)
    // This does not stop in simulation because encoders don't get values...
    public Command autoMidCube() {
        return this.commandBuilder
                .primeCubeMid()
                .andThen(autoCubeOut())
                .andThen(this.armSubsystem.homeArmCommand());
    }

    public Command generateAutonomous(String path) {
        final List<PathPlannerTrajectory> pathGroup;
        HashMap<String, Command> eventMap = new HashMap<>();
        // This will load the file "FullAuto.path"
        // All paths are in /src/main/deploy/pathplanner
        // Please set robot width/length in PathPlanner to 34 x 34 inches --> meters (0.8636 meters)
        eventMap.put("autoCubeMid", this.autoMidCube());
        eventMap.put("balanceChargeStation", this.balanceChargeStation());
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
                pathGroup = PathPlanner.loadPathGroup("Nothing", new PathConstraints(2, 1.5));
                break;
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
}
