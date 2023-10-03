package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class TeleopSwerve extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier translationSup;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;
    private final DoubleSupplier rotationPov;
    private final Logger logger = Logger.getInstance();

    private final Set<Double> allowedAngles = Set.of(0., 90., 180., 270.);

    public TeleopSwerve(
            DriveSubsystem s_Swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            DoubleSupplier rotationPov) {
        this.driveSubsystem = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.rotationPov = rotationPov;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal =
                MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeVal =
                MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
        double rotationVal =
                MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND);

        logger.recordOutput("DriveCommand/translation", translationVal);
        logger.recordOutput("DriveCommand/strafe", strafeVal);
        logger.recordOutput("DriveCommand/rot", rotationVal);

        if (!DriverStation.isAutonomous()) {
            /* Drive */
            driveSubsystem.angleDrive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                    rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                    -rotationPov.getAsDouble(),
                    !robotCentricSup.getAsBoolean(),
                    true,
                    true,
                    allowedAngles.contains(rotationPov.getAsDouble()));
        }
    }
}
