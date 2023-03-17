package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.arm.ArmSubsystem;

import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase implements BlitzSubsystem {
    private final WristIO io;

    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private final Logger logger = Logger.getInstance();

    public WristSubsystem(WristIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        logger.processInputs("wrist", inputs);
    }

    public void setRotationSpeed(double speed) {
        io.setRotationSpeed(speed);
    }

    public CommandBase rotateToCommand(double rotation, double threshold, double goal) {
       /*  return Commands.run(() -> {.armSubsystem
            armSubsystem.getState().rotation;
            io.setRotation(1);
        }, this); */
        return WristSubsystem.getState().rotation > goal - threshold
                && WristSubsystem.getState().rotation < goal + threshold;
    }
}