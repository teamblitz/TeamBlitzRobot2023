package frc.lib;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface BlitzSubsystem extends Subsystem {
    /** Initializes the telemetry for this subsystem. */
    default void initTelemetry() {}
}
