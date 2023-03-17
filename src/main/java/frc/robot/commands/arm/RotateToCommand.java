package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class RotateToCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final double goal;
    private final double threshold;

    private TrapezoidProfile profile;

    private double lastTime;

    public RotateToCommand(ArmSubsystem armSubsystem, double goal, double threshold) {
        this.armSubsystem = armSubsystem;
        this.goal = goal;
        this.threshold = threshold;
    }

    @Override
    public void initialize() {
        profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(Constants.Arm.VELOCITY_METERS_PER_SECOND, Constants.Arm.ACCELERATION_METERS_PER_SECOND_SQUARED),
                new TrapezoidProfile.State(armSubsystem.getRotation(), armSubsystem.getRotationSpeed()),
                new TrapezoidProfile.State(goal, 0)
        );
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double deltaTime = Timer.getFPGATimestamp() - lastTime;
        lastTime = Timer.getFPGATimestamp();
        TrapezoidProfile.State state = profile.calculate(deltaTime);
        armSubsystem.updateRotation(state.position, state.velocity);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.getState().rotation > goal - threshold
                && armSubsystem.getState().rotation < goal + threshold;
    }
}
