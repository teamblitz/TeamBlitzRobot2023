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

    private double startTime;

    public RotateToCommand(ArmSubsystem armSubsystem, double goal, double threshold) {
        this.armSubsystem = armSubsystem;
        this.goal = goal;
        this.threshold = threshold;

        addRequirements(armSubsystem.RotationRequirement);
    }

    @Override
    public void initialize() {
        profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                Constants.Arm.ROTATION_VELOCITY,
                                Constants.Arm.ROTATION_ACCELERATION),
                        new TrapezoidProfile.State(goal, 0),
                        new TrapezoidProfile.State(
                                armSubsystem.getRotation(), 0));
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        TrapezoidProfile.State state = profile.calculate(Timer.getFPGATimestamp() - startTime);
        armSubsystem.updateRotation(state.position, 0);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.getRotation() > goal - threshold
                && armSubsystem.getRotation() < goal + threshold;
    }
}
