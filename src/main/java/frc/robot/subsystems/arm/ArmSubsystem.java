package frc.robot.subsystems.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.lib.math.controller.TelescopingArmFeedforward;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.atomic.AtomicReference;

public class ArmSubsystem extends SubsystemBase implements BlitzSubsystem {

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private final Logger logger = Logger.getInstance();

    private final TelescopingArmFeedforward rotationFeedforward;

    public ArmSubsystem(ArmIO io) {
        this.io = io;

        rotationFeedforward =
                new TelescopingArmFeedforward(
                        (x) -> 0., (x) -> 0., (x) -> 0.,
                        (x) -> 0.); // TODO: Make these actual gains
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        logger.processInputs("arm", inputs);
    }

    public void updateRotation(double degrees, double velocity) {
        io.setRotationSetpoint(degrees, rotationFeedforward.calculate(inputs.armExtension, Math.toRadians(degrees), velocity));
    }

    public ArmState getState() {
        return new ArmState(inputs.armRot, inputs.armExtension);
    }

    public double getRotation() {
        return inputs.armRot;
    }

    public double getRotationSpeed() {
        return inputs.armSpeed;
    }

    public void setArmRotationSpeed(double percent) {
        io.setArmRotationSpeed(percent);
    }

    public void setArmExtensionSpeed(double percent) {
        io.setArmExtensionSpeed(percent);
    }


    public boolean validRot(double degrees) {
        return true;
    }

    // TODO: Move to own class or delete
    /** A data class representing a possible state for the arm. */
    public static class ArmState {
        /** Arm rotation in degrees where horizontal is 0, increasing as the arm is raised. */
        public final double rotation;
        /** Arm extension in meters where 0 is not extended */
        public final double extension;

        public ArmState(double rotation, double extension) {
            this.rotation = rotation;
            this.extension = extension;
        }

        /**
         * Checks if the state is within the arms possible range of motion.
         *
         * @return True if the arm state is within its possible range of motion.
         */
        public boolean isValid() {
            return true;
        }

        /**
         * Checks if the arm is outside the legal height and extension range of the robot. Does not
         * check if the state is valid, just that it is legal.
         *
         * @return True if the arm state is within the extension range of the robot.
         */
        public boolean isSafe() {
            return true;
        }
    }
}
