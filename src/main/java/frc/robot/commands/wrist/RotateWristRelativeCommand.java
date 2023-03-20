package frc.robot.commands.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RotateWristRelativeCommand extends CommandBase {
    private final WristSubsystem wristSubsystem;

    private final double goal;
    private final double threshold;

    private TrapezoidProfile profile;

    private double lastTime;

    public RotateWristRelativeCommand(
            WristSubsystem wristSubsystem, double goal, double threshold) {
        this.wristSubsystem = wristSubsystem;

        this.goal = goal;
        this.threshold = threshold;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.wristSubsystem);
    }

    @Override
    public void initialize() {
        profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                Constants.Arm.ROTATION_VELOCITY,
                                Constants.Arm.ROTATION_ACCELERATION),
                        new TrapezoidProfile.State(
                                wristSubsystem.getRelativeRotation(),
                                wristSubsystem.getRotationSpeed()),
                        new TrapezoidProfile.State(goal, 0));
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double deltaTime = Timer.getFPGATimestamp() - lastTime;
        lastTime = Timer.getFPGATimestamp();
        TrapezoidProfile.State state = profile.calculate(deltaTime);
        wristSubsystem.updateRelativeRotation(state.position, state.velocity);
    }

    @Override
    public boolean isFinished() {
        return wristSubsystem.getRelativeRotation() > goal - threshold
                && wristSubsystem.getRelativeRotation() < goal + threshold;
    }
}
