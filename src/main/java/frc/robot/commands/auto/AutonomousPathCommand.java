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
    private final Command fullAuto;

    public AutonomousPathCommand(
            final DriveSubsystem driveSubsystem,
            final ArmSubsystem armSubsystem,
            final IntakeSubsystem intakeSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and
        // a max acceleration of 3 m/s^2
        // for every path in the group
        // All paths are in /src/main/deploy/pathplanner
        // Please set robot width/length in PathPlanner to 34 x 34 inches --> meters (0.8636 meters)

        // <-- Initially an ArrayList... may cause errors later -->
        List<PathPlannerTrajectory> pathGroup =
                PathPlanner.loadPathGroup("SquarePath", new PathConstraints(2, 1.5));

        HashMap<String, Command> eventMap = new HashMap<>();
        // Use Stop Markers instead of markers because markers run in parallel
        // Use none
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        eventMap.put("marker2", new PrintCommand("Passed marker 2"));
        eventMap.put("outCube", this.autoCubeOut());
        eventMap.put("inCube", this.autoCubeIn());
        // eventMap.put("marker3", new ExtendToCommand(this.armSubsystem, 0, 0));
        // eventMap.put("marker4", new RotateToCommand(this.armSubsystem, 0, 0));

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not
        // every time you want to create an auto command. A good place to put this is in
        // RobotContainer along with your subsystems.
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
                        (states) -> this.driveSubsystem.setModuleStates(states, false, false),
                        eventMap,
                        // Should the path be automatically mirrored depending on alliance color
                        true,
                        this.driveSubsystem);
        this.fullAuto = autoBuilder.fullAuto(pathGroup);
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
        return Commands.run(() -> this.intakeSubsystem.inCone(), intakeSubsystem)
                .withTimeout(2)
                .andThen(() -> this.intakeSubsystem.stop(), intakeSubsystem);
    }

    public Command autoConeOut() {
        return Commands.run(() -> this.intakeSubsystem.outCone(), intakeSubsystem)
                .withTimeout(0.25)
                .andThen(
                        () -> this.intakeSubsystem.stop(),
                        intakeSubsystem); // No need for the stop because intake commands will turn
        // off the motor.
    }

    public Command autoCubeIn() {
        return Commands.run(() -> this.intakeSubsystem.inCube(), intakeSubsystem)
                .withTimeout(2)
                .andThen(() -> this.intakeSubsystem.stop(), intakeSubsystem);
    }

    public Command autoCubeOut() {
        return Commands.run(() -> this.intakeSubsystem.outCube(), intakeSubsystem)
                .withTimeout(0.25)
                .andThen(() -> this.intakeSubsystem.stop(), intakeSubsystem);
    }

    // Arm/Wrist Commands
    public Command autoExtendArm(double distance) {
        return null;
    }

    public Command emergencyStop() {
        this.armSubsystem.setArmExtensionSpeed(0);
        this.armSubsystem.setArmRotationSpeed(0);
//        this.armSubsystem.setWristRotationSpeed(0);

        return Commands.run(
                () -> this.driveSubsystem.drive(new Translation2d(0, 0), 0, false, true, false),
                driveSubsystem); // .alongWith...? Or just make this set everything to 0 no command
        // just void
    }

    public Command getFullAuto() {
        return this.fullAuto;
    }
}
