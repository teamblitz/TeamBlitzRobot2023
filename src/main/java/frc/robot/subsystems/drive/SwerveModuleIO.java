package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleInputs {
        public double speedMetersPerSecond;
        public double drivePositionMeters;
        public double anglePositionDegrees;
        public double absoluteEncoderPositionDegrees;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(SwerveModuleInputs inputs) {}

    public default void setDrivePercent(double percent) {}

    /**
     * Sets the velocity setpoint for the drive pid controller
     *
     * @param setpoint speed in meters per second.
     * @param ffVolts Feed forward in volts.
     */
    public default void setDriveSetpoint(double setpoint, double ffVolts) {}

    /**
     * Sets the position setpoint for the angle pid controller
     *
     * @param setpoint position in degrees
     */
    public default void setAngleSetpoint(double setpoint) {}

    /** Configure the PID constants for the drive controller */
    public default void configureDrivePID(double p, double i, double d) {}

    /** Configure the PID constants for the angle controller */
    public default void configureAnglePID(double p, double i, double d) {}

    public default void setBrakeMode(boolean enabled) {}
}
