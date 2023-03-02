package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class RotateToCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final double goal;

    public RotateToCommand(ArmSubsystem armSubsystem, double goal) {
        this.armSubsystem = armSubsystem;
        this.goal = goal;
    }

    @Override
    public void initialize() {
        armSubsystem.rotateTo(goal);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.getState().rotation > 2 * goal - 4
                && armSubsystem.getState().rotation < 2 * goal;
    }
}
