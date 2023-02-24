package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    /** All units are meters and degrees */
    @AutoLog
    public class ArmIOInputs {
        public double armRot;
        public double armExtension;
        public double wristRot;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setArmRotation(double degrees) {}

    public default void setArmExtension(double meters) {}

    public default void setWristRotation(double rot) {}

    public default void setArmRotationSpeed(double speed) {}

    public default void setArmExtensionSpeed(double speed) {}

    public default void setWristRotationSpeed(double speed) {}
}
