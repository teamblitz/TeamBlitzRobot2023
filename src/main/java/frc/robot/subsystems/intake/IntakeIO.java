package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public class IntakeIOInputs {}

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void inCone() {}

    public default void outCone() {}

    public default void inCube() {}

    public default void outCube() {}

    public default void stop() {}
}
