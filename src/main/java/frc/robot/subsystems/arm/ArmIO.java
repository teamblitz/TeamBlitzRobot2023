package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    /** All units are meters and degrees */
    @AutoLog
    public class ArmIOInputs {
        public double armRot;
        public double absArmRot;
        public double armExtension;
        public double wristRot;
        public double absWristRot;

        public boolean topRotationLimit;
        public boolean bottomRotationLimit;
        public boolean topWristLimit;
        public boolean bottomWristLimit;
        public boolean maxExtensionLimit;
        public boolean minExtensionLimit;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setArmRotation(double degrees) {}

    public default void setArmExtension(double meters) {}

    public default void setWristRotation(double rot) {}

    public default void setArmRotationSpeed(double speed) {}

    public default void setArmExtensionSpeed(double speed) {}

    public default void setLeftExtensionSpeed(double speed) {}

    public default void setRightExtensionSpeed(double speed) {}

    public default void setWristRotationSpeed(double speed) {}
}
