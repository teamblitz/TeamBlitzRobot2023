package frc.robot.commands.arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ExtendToCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final double goal;
    private final double threshold;

    private TrapezoidProfile profile;

    private double lastTime;

    public ExtendToCommand(ArmSubsystem armSubsystem, double goal, double threshold) {
        this.armSubsystem = armSubsystem;
        this.goal = goal;
        this.threshold = threshold;

        addRequirements(armSubsystem.ExtensionRequirement);
    }

    @Override
    public void initialize() {
        profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                Constants.Arm.EXTENSION_VELOCITY,
                                Constants.Arm.EXTENSION_ACCELERATION),
                        new TrapezoidProfile.State(
                                armSubsystem.getExtension(), armSubsystem.getExtensionSpeed()),
                        new TrapezoidProfile.State(goal, 0));
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double deltaTime = Timer.getFPGATimestamp() - lastTime;
        lastTime = Timer.getFPGATimestamp();
        TrapezoidProfile.State state = profile.calculate(deltaTime);
        armSubsystem.updateExtension(state.position, state.velocity);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.getExtension() > goal - threshold
                && armSubsystem.getExtension() < goal + threshold;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setArmExtensionSpeed(
                0); // The arm can hold itself extension wise, so no need to waste motor power.
    }
}
