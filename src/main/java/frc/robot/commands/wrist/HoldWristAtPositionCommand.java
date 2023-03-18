package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.WristSubsystem;

public class HoldWristAtPositionCommand extends CommandBase {

    private final WristSubsystem wristSubsystem;

    private double initialPosition;

    public HoldWristAtPositionCommand(WristSubsystem wristSubsystem) {
        this.wristSubsystem = wristSubsystem;
    }

    @Override
    public void initialize() {
        initialPosition = wristSubsystem.getRotation();

        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        wristSubsystem.updateRelativeRotation(initialPosition, 0);
    }
}
