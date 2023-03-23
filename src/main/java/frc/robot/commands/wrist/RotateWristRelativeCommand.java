package frc.robot.commands.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.wrist.WristSubsystem;
import org.littletonrobotics.junction.Logger;

public class RotateWristRelativeCommand extends CommandBase {
    private final WristSubsystem wristSubsystem;

    private final double goal;
    private final double threshold;

    private TrapezoidProfile profile;

    private double startTime;

    public RotateWristRelativeCommand(
            WristSubsystem wristSubsystem, double goal, double threshold) {
        this.wristSubsystem = wristSubsystem;

        this.goal = goal;
        Logger.getInstance().recordOutput("wrist/commands/goal", goal);
        this.threshold = threshold;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.wristSubsystem);
    }

    @Override
    public void initialize() {
        Logger.getInstance()
                .recordOutput("wrist/commands/rot_rel", wristSubsystem.getRelativeRotation());
        Logger.getInstance()
                .recordOutput("wrist/commands/speed", wristSubsystem.getRotationSpeed());
        profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                Constants.Wrist.ROTATION_VELOCITY,
                                Constants.Wrist.ROTATION_ACCELERATION),
                        new TrapezoidProfile.State(goal, 0),
                        new TrapezoidProfile.State(
                                wristSubsystem.getRelativeRotation(),
                                wristSubsystem.getRotationSpeed()));
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State state = profile.calculate(Timer.getFPGATimestamp() - startTime);
        wristSubsystem.updateRelativeRotation(state.position, state.velocity);
    }

    @Override
    public boolean isFinished() {
        return wristSubsystem.getRelativeRotation() > goal - threshold
                && wristSubsystem.getRelativeRotation() < goal + threshold;
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.lastRelativeGoal = goal;
        wristSubsystem.lastGoal = wristSubsystem.getRotation();
    }
}
