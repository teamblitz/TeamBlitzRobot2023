package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public class ArmIOInputs {}

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {}

    public default void setArmRotation(Rotation2d rot) {}

    public default void setArmPosition(double meters) {}

    public default void setWristRotation(Rotation2d rot) {}

    public default void setArmRotationSpeed(double speed) {}

    public default void setArmExtensionSpeed(double speed) {}

    public default void setWristRotationSpeed(double speed) {}
}
