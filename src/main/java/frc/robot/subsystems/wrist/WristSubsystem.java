package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase implements BlitzSubsystem {
    private final WristIO io;

    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private final ArmSubsystem armSubsystem;

    private final ArmFeedforward feedforward;

    private final Logger logger = Logger.getInstance();

    public WristSubsystem(WristIO io, ArmSubsystem armSubsystem) {
        this.io = io;
        this.armSubsystem = armSubsystem;
        // The wrist is basically an arm, so we treat it as such.
        feedforward =
                new ArmFeedforward(Constants.Wrist.ks, Constants.Wrist.kg, Constants.Wrist.kv);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        logger.processInputs("wrist", inputs);
    }

    public void setRotationSpeed(double speed) {
        io.setRotationSpeed(speed);
    }

    /**
     * Update the onboard pid controller based off of the relative angle of the arm
     *
     * @param relativeRot Rotation relative to the robot in degrees
     * @param velocity Velocity in degrees per second
     */
    public void updateRelativeRotation(double relativeRot, double velocity) {
        // Arm Rot + Wrist Rot = Relative Wrist Rot
        // Wrist rot = Relative Wrist Rot - arm rot
        double rot = relativeRot - armSubsystem.getRotation();
        double clamped =
                MathUtil.clamp(rot, Constants.Wrist.MIN_ROTATION, Constants.Wrist.MAX_ROTATION);
        // Don't do feedforward if we are clamping, so we don't push into ourselves
        io.setRotationSetpoint(
                clamped,
                rot != clamped
                        ? feedforward.calculate(
                                Math.toRadians(relativeRot), Math.toRadians(velocity))
                        : 0);
    }

    public double getRotation() {
        return inputs.rotation;
    }

    public double getRelativeRotation() {
        return armSubsystem.getRotation() + getRotation();
    }

    public double getRotationSpeed() {
        return inputs.rotationSpeed;
    }

    public CommandBase rotateToCommand(double rotation, double threshold, double goal) {
        /*  return Commands.run(() -> {.armSubsystem
            armSubsystem.getState().rotation;
            io.setRotation(1);
        }, this); */
        //        return WristSubsystem.getState().rotation > goal - threshold
        //                && WristSubsystem.getState().rotation < goal + threshold;
        return null;
    }
}
