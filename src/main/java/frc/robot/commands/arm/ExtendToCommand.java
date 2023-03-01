package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ExtendToCommand extends CommandBase{

    private final ArmSubsystem armSubsystem;
    private final double goal;

    public ExtendToCommand(ArmSubsystem armSubsystem, double goal)  {
        this.armSubsystem = armSubsystem;
        this.goal = goal;
    }

    @Override
    public void initialize() {
        armSubsystem.goTo(null);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.getState().rotation > goal - 4 && armSubsystem.getState().rotation < goal + 4;
    }
}
