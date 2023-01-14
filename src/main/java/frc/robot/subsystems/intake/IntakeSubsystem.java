package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;

public class IntakeSubsystem extends SubsystemBase implements BlitzSubsystem {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public void coneIn() {}

    public void coneOut() {}

    public void cubeIn() {}

    public void cubeOut() {}

    public void stop() {
        io.setFront(0);
        io.setBack(0);
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public State getState() {
        return State.UNKNOWN;
    }

    public enum State {
        CUBE,
        CONE,
        NONE,
        UNKNOWN
    }
}
