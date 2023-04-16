package frc.robot.commands.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.wrist.WristSubsystem;
import org.littletonrobotics.junction.Logger;

// TODO: Make inlined
public class RotateWristCommand extends CommandBase {
    private final WristSubsystem wristSubsystem;

    private final double goal;
    private final double threshold;

    private TrapezoidProfile profile;

    private double startTime;

    public RotateWristCommand(WristSubsystem wristSubsystem, double goal, double threshold) {
        this.wristSubsystem = wristSubsystem;

        this.goal = goal;
        this.threshold = threshold;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.wristSubsystem);
    }

    @Override
    public void initialize() {
        Logger.getInstance().recordOutput("wrist/rotate_to_active", true);
        profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                Constants.Wrist.ROTATION_VELOCITY,
                                Constants.Wrist.ROTATION_ACCELERATION),
                        new TrapezoidProfile.State(goal, 0),
                        new TrapezoidProfile.State(
                                wristSubsystem.getRotation(), wristSubsystem.getRotationSpeed()));
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State state = profile.calculate(Timer.getFPGATimestamp() - startTime);
        wristSubsystem.updateRotation(state.position, state.velocity);
    }

    @Override
    public boolean isFinished() {
        return wristSubsystem.getRotation() > goal - threshold
                && wristSubsystem.getRotation() < goal + threshold;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.getInstance().recordOutput("wrist/rotate_to_active", false);
        wristSubsystem.lastGoal = goal;
        wristSubsystem.lastRelativeGoal = wristSubsystem.getRelativeRotation();
    }
}
