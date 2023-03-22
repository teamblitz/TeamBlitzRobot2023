package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    /** All units are meters and degrees */
    @AutoLog
    public class WristIOInputs {
        public double rotation;
        public double absoluteRotation;

        public double absEncoder;
        public double rotationSpeed;

        public boolean topLimit;
        public boolean bottomLimit;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(WristIOInputs inputs) {}

    public default void setRotationSetpoint(double rot, double arbFFVolts) {}

    public default void setRotationSpeed(double speed) {}

    public default void setVoltage(double voltage) {}

    public default void setPID(double p, double i, double d) {}

    public default void seedWristPosition(boolean assumeStarting) {}

    public default void checkLimitSwitches() {}
}
