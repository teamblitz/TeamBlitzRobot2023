package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;

public class ArmSubsystem extends SubsystemBase implements BlitzSubsystem {
    public ArmSubsystem() {}

    public ArmState getPosition() {
        return null;
    }

    public void goTo(ArmState state) {}

    /** A data class representing a possible state for the arm. */
    public static class ArmState {
        /** Arm rotation where horizontal is 0, increasing as the arm is raised. */
        public final Rotation2d rotation;
        /** Arm extension in meters where 0 is not extended */
        public final double extension;

        public ArmState(Rotation2d rotation, double extension) {
            this.rotation = rotation;
            this.extension = extension;
        }

        /**
         * Checks if the state is within the arms possible range of motion.
         *
         * @return True if the arm state is within its possible range of motion.
         */
        public boolean isValid() {
            return false;
        }

        /**
         * Checks if the arm is outside the legal height and extension range of the robot. Does not
         * check if the state is valid, just that it is legal.
         *
         * @return True if the arm state is within the extension range of the robot.
         */
        public boolean isSafe() {
            return false;
        }
    }
}
