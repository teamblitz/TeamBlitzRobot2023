package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase implements BlitzSubsystem {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final Logger logger = Logger.getInstance();

    private boolean gamePiceInIntakeFlag;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public void inCone() {
        io.inCone();
    }

    public void outCone() {
        io.outCone();
    }

    public void inCube() {
        io.inCube();
    }

    public void outCube() {
        io.outCube();
    }

    public void stop() {
        gamePiceInIntakeFlag = false;
        io.stop();
    }

    public void slowIn() {
        io.set(Constants.Intake.Simple.SLOW_INTAKE);
    }

    public void periodic() {
        io.updateInputs(inputs);
        logger.processInputs("intake", inputs);
        logger.recordOutput("intake/hasGampiece", gamePieceInIntake());
    }

    public State getState() {
        return State.UNKNOWN;
    }

    public Command buildCubeInCommand() {
        return Commands.startEnd(this::inCube, this::stop, this);
    }

    public Command buildCubeOutCommand() {
        return Commands.startEnd(this::outCube, this::stop, this);
    }

    public Command buildConeInCommand() {
        return Commands.startEnd(this::inCone, this::stop, this);
    }

    public Command buildConeOutCommand() {
        return Commands.startEnd(this::outCone, this::stop, this);
    }

    public Command slowIntakeCommand() {
        return Commands.startEnd(this::slowIn, this::stop, this);
    }

    public enum State {
        CUBE,
        CONE,
        NONE,
        UNKNOWN
    }

    private boolean gamePieceInIntake() {
        // TODO: CHANGED TO MAKE BUILD
        if (!gamePiceInIntakeFlag
                && inputs.current > Constants.Intake.DETECTION_CURRENT_THRESHOLD) {
            gamePiceInIntakeFlag = true;
            return true;
        }
        return gamePiceInIntakeFlag;
    }
}
