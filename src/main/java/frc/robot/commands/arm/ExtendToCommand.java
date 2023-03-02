package frc.robot.commands.arm;

import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ExtendToCommand extends CommandBase{

    private final ArmSubsystem armSubsystem;
    private final double goal;
    private final double threshold;

    public ExtendToCommand(ArmSubsystem armSubsystem, double goal, double threshold)  {
        this.armSubsystem = armSubsystem;
        this.goal = goal;
        this.threshold = threshold;
    }

    @Override
    public void initialize() {
        armSubsystem.extendTo(goal);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.getState().extension > goal - threshold && armSubsystem.getState().extension < goal + threshold;
    }
}
