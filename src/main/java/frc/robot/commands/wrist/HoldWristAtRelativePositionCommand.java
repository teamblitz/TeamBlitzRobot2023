package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.wrist.WristSubsystem;

public class HoldWristAtRelativePositionCommand extends CommandBase {

    private final WristSubsystem wristSubsystem;

    private double initialPosition;

    public HoldWristAtRelativePositionCommand(WristSubsystem wristSubsystem) {
        this.wristSubsystem = wristSubsystem;

        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        initialPosition = wristSubsystem.lastRelativeGoal;
    }

    @Override
    public void execute() {
        wristSubsystem.updateRelativeRotation(initialPosition, 0);
    }
}
