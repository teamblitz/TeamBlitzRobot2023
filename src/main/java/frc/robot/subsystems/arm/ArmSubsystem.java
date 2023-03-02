package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements BlitzSubsystem {

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private final Logger logger = Logger.getInstance();

    private double wantedWristRot;

    public ArmSubsystem(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        logger.processInputs("arm", inputs);

        //        if (wantedWristRot - inputs.armRot > 90 || inputs.armRot < -90) {
        //            io.setWristRotation(wantedWristRot - inputs.armRot);
        //        }
    }

    public void goTo(ArmState state) {
        if (!state.isValid() || !state.isSafe()) {
            return;
        }
        io.setArmRotation(state.rotation);
        io.setArmExtension(state.extension);
    }

    public void rotateTo(double degrees) {
        // TODO: Make sure it is valid.
        io.setArmRotation(degrees);
    }

    public ArmState getState() {
        return new ArmState(inputs.armRot, inputs.armExtensionL);
    }

    public void setWristRot(double degrees) {
        if (degrees < 0 || degrees > 90) {
            return;
        }
        wantedWristRot = degrees;
    }

    public void setArmRotationSpeed(double percent) {
        double percent2 = 0;
        if (percent != 0) {
            if (Math.abs(percent) == percent) {
                percent2 = percent * 90 - Math.max(inputs.armRot, 0) * (1.0 / 90.0);
            } else {
                percent2 = percent * Math.min(inputs.armRot, 90) * (1.0 / 90.0);
            }
        }
        io.setArmRotationSpeed(percent);
    }

    public void setArmExtensionSpeed(double percent) {
        io.setArmExtensionSpeed(percent);
    }

    public void setWristRotationSpeed(double percent) {
        io.setWristRotationSpeed(percent);
    }

    // TODO: Make this more specific to left and right over leader/follower.
    public void setLeftExtensionSpeed(double percent) {
        io.setRightExtensionSpeed(percent);
    }

    public void setRightExtensionSpeed(double percent) {
        io.setLeftExtensionSpeed(percent);
    }

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
