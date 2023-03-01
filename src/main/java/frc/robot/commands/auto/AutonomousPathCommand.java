package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;

public class AutonomousPathCommand extends SequentialCommandGroup {
    private final DriveSubsystem driveSubsystem;
    private final GyroIOPigeon gyroIOPigeon;
    private final ArmSubsystem armSubsystem;

    public AutonomousPathCommand(
            final DriveSubsystem driveSubsystem,
            final GyroIOPigeon gyroIOPigeon,
            final ArmSubsystem armSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.gyroIOPigeon = gyroIOPigeon;
        this.armSubsystem = armSubsystem;
    }

    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            // Reset odometry for the first path you run during auto
                            if (isFirstPath) {
                                driveSubsystem.resetOdometry(traj.getInitialHolonomicPose());
                            }
                        }),
                new PPSwerveControllerCommand(
                        traj,
                        driveSubsystem::getPose, // Pose supplier
                        driveSubsystem.getKinematics(), // SwerveDriveKinematics
                        new PIDController(
                                0, 0,
                                0), // X controller. Tune these values for your robot. Leaving them
                        // 0 will only use feedforwards.
                        new PIDController(
                                0, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(
                                0, 0, 0), // Rotation controller. Tune these values for your robot.
                        // Leaving them 0 will only use feedforwards.
                        (states) ->
                                driveSubsystem.setModuleStates(
                                        states, false, false), // Module states consumer
                        true, // Should the path be automatically mirrored depending on alliance
                        // color. Optional, defaults to true
                        driveSubsystem // Requires this drive subsystem
                        ));
    }
}
