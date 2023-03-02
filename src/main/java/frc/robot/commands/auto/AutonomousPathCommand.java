package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.HashMap;
import java.util.List;

// If the code is bleeding, try running a gradle.bat spotlessApply and then deploying it
// This class is mainly just storage for all the other commands we'll use in autonomous or to order
// them
public class AutonomousPathCommand {
    private final DriveSubsystem driveSubsystem;
    private final ArmSubsystem armSubsystem;
    private final Command fullAuto;

    public AutonomousPathCommand(
            final DriveSubsystem driveSubsystem, final ArmSubsystem armSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and
        // a max acceleration of 3 m/s^2
        // for every path in the group
        // Except I changed it to "SquarePath.path" with a max velocity of 2 m/s and acceleration of
        // 1.5 m/s
        // All paths are in /src/main/deploy/pathplanner
        // Please set robot width/length in PathPlanner to 34 x 34 inches --> meters (0.8636 meters)
        // <-- Initially an ArrayList... may cause errors later -->
        List<PathPlannerTrajectory> pathGroup =
                PathPlanner.loadPathGroup("SquarePath", new PathConstraints(2, 1.5));

        // This is just an example event map. It would be better to have a constant, global event
        // map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("marker2", new PrintCommand("Passed marker 2"));
        // eventMap.put("intakeDown", new IntakeDown());

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not
        // every time you want to create an auto command. A good place to put this is in
        // RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder =
                new SwerveAutoBuilder(
                        driveSubsystem::getPose, // Pose2d supplier
                        driveSubsystem
                                ::resetOdometry, // Pose2d consumer, used to reset odometry at the
                        // beginning of auto (this is assuming that reset Odometry is the same as
                        // resetPose)
                        // Please fix this later at some point lol  
                        Constants.Swerve.KINEMATICS, // SwerveDriveKinematics
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
                        (states) ->
                                driveSubsystem.setModuleStates(
                                        states, false,
                                        false), // Module states consumer used to output to the
                        // drive subsystem
                        eventMap,
                        true, // Should the path be automatically mirrored depending on alliance
                        // color. Optional, defaults to true
                        driveSubsystem);
        this.fullAuto = autoBuilder.fullAuto(pathGroup);
    }

    public Command getFullAuto() {
        return this.fullAuto;
    }
}

// Don't delete this yet please it's a last resort
// Assuming this method is part of a drivetrain subsystem that provides the necessary methods
// It's not, but it's modified to work here
//     public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
//         return new SequentialCommandGroup(
//                 new InstantCommand(
//                         () -> {
//                             // Reset odometry for the first path you run during auto
//                             if (isFirstPath) {
//                                 driveSubsystem.resetOdometry(traj.getInitialHolonomicPose());
//                             }
//                         }),
//                 new PPSwerveControllerCommand(
//                         traj,
//                         driveSubsystem::getPose, // Pose supplier
//                         driveSubsystem.getKinematics(), // SwerveDriveKinematics
//                         new PIDController(
//                                 0, 0, 0), // X controller. Tune these values for your robot.
// Leaving them
//                         // 0 will only use feedforwards.
//                         new PIDController(
//                                 0, 0, 0), // Y controller (usually the same values as X
// controller)
//                         new PIDController(
//                                 0, 0, 0), // Rotation controller. Tune these values for your
// robot.
//                         // Leaving them 0 will only use feedforwards.
//                         (states) ->
//                                 driveSubsystem.setModuleStates(
//                                         states, false, false), // Module states consumer
//                         true, // Should the path be automatically mirrored depending on alliance
//                         // color. Optional, defaults to true
//                         driveSubsystem // Requires this drive subsystem
//                         ));
//     }
