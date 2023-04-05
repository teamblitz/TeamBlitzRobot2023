package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    /** All units are meters and degrees */
    @AutoLog
    public class ArmIOInputs {
        public double armRot;
        public double armRotationSpeed;
        public double absArmRot;

        public double absArmEncoder;
        public double armExtension;
        public double armExtensionSpeed;

        public boolean topRotationLimit;
        public boolean bottomRotationLimit;
        public boolean maxExtensionLimit;
        public boolean minExtensionLimit;

        public boolean encoderConnected;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setRotationSetpoint(double degrees, double arbFFPercent) {}

    public default void setExtensionSetpoint(double meters) {}

    public default void setArmRotationSpeed(double percent) {}

    public default void setArmExtensionSpeed(double percent) {}

    public default void seedArmPosition() {}

    public default void checkLimitSwitches() {}
}
