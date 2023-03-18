package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class HoldArmAtPositionCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    private double initialPosition;

    public HoldArmAtPositionCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        initialPosition = armSubsystem.getRotation();

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.updateRotation(initialPosition, 0);
    }
}
