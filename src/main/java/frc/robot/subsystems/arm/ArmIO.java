package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    /** All units are meters and degrees */
    @AutoLog
    public class ArmIOInputs {
        public double armRot;
        public double armSpeed;
        public double absArmRot;
        public double armExtension;

        public boolean topRotationLimit;
        public boolean bottomRotationLimit;
        public boolean maxExtensionLimit;
        public boolean minExtensionLimit;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setRotationSetpoint(double degrees, double arbFFPercent) {}

    public default void setArmExtension(double meters) {}

    public default void setArmRotationSpeed(double speed) {}

    public default void setArmExtensionSpeed(double speed) {}

    public default void resetToAbsolute() {}

    public default void checkLimitSwitches() {}
}
