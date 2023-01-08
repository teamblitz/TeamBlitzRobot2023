package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.List;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(DriveSubsystem s_Swerve) {
        TrajectoryConfig config =
                new TrajectoryConfig(
                                Constants.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
                                Constants.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                        .setKinematics(Constants.Swerve.KINEMATICS);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        config);

        var thetaController =
                new ProfiledPIDController(
                        Constants.AutoConstants.kPThetaController,
                        0,
                        0,
                        Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                        exampleTrajectory,
                        s_Swerve::getPose,
                        Constants.Swerve.KINEMATICS,
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                        thetaController,
                        (x) -> s_Swerve.setModuleStates(x, true, false),
                        s_Swerve);

        addCommands(
                new InstantCommand(
                        () -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
                swerveControllerCommand);
    }
}
