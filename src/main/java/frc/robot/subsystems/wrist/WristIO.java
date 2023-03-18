package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    /** All units are meters and degrees */
    @AutoLog
    public class WristIOInputs {
        public double rotation;
        public double absoluteRotation;
        public double rotationSpeed;

        public boolean topLimit;
        public boolean bottomLimit;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(WristIOInputs inputs) {}

    public default void setRotationSetpoint(double rot, double arbFFPercent) {}

    public default void setRotationSpeed(double speed) {}
}
