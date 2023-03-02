package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class RotateToCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final double goal;
    private final double threshold;

    public RotateToCommand(ArmSubsystem armSubsystem, double goal, double threshold) {
        this.armSubsystem = armSubsystem;
        this.goal = goal;
        this.threshold = threshold;
    }

    @Override
    public void initialize() {
        armSubsystem.rotateTo(goal);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.getState().rotation > goal - threshold && armSubsystem.getState().rotation < goal + threshold;
    }
}
