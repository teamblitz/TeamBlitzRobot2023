/* Big thanks to Team 364 for the base code. */

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.ModuleStateOptimizer;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    public final int moduleNumber;
    private final SwerveModuleIO io;
    private final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    private Rotation2d lastAngle;

    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    Constants.Swerve.DRIVE_KS,
                    Constants.Swerve.DRIVE_KV,
                    Constants.Swerve.DRIVE_KA);
    private final String logKey;

    private final Logger logger = Logger.getInstance();

    public SwerveModule(int moduleNumber, SwerveModuleIO io) {
        this.io = io;
        this.moduleNumber = moduleNumber;

        lastAngle = getAngle();

        logKey = "Swerve/Mod" + moduleNumber;
    }

    public void configAnglePid(double p, double i, double d) {
        io.configureAnglePID(p, i, d);
    }

    public void configDrivePid(double p, double i, double d) {
        io.configureDrivePID(p, i, d);
    }

    public void periodic() {
        io.updateInputs(inputs);
        logger.processInputs(logKey, inputs);
    }

    public void setDesiredState(
            SwerveModuleState desiredState, boolean isOpenLoop, boolean tuning, boolean parking) {

        desiredState = ModuleStateOptimizer.optimize(desiredState, getState().angle);

        setAngle(desiredState, tuning, parking);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            io.setDrivePercent(percentOutput);
            logger.recordOutput(logKey + "/drivePercent", percentOutput);
        } else {
            logger.recordOutput(logKey + "/speedSetpoint", desiredState.speedMetersPerSecond);
            io.setDriveSetpoint(
                    desiredState.speedMetersPerSecond,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean tuning, boolean parking) {
        Rotation2d angle =
                (!(tuning || parking)
                                && Math.abs(desiredState.speedMetersPerSecond)
                                        <= (Constants.Swerve.MAX_SPEED * 0.01))
                        ? lastAngle
                        : desiredState.angle; // Prevent rotating module if speed is less than 1%.
        io.setAngleSetpoint(angle.getDegrees());
        logger.recordOutput(logKey + "/AngleSetpoint", angle.getDegrees());
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(inputs.anglePositionDegrees);
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(inputs.absoluteEncoderPositionDegrees);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(inputs.speedMetersPerSecond, getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(inputs.drivePositionMeters, getAngle());
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }
}
