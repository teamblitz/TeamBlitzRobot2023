package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ScoreInitialGamePieceCommand extends SequentialCommandGroup {
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ScoreInitialGamePieceCommand(
            final ArmSubsystem armSubsystem, final IntakeSubsystem intakeSubsystem) {
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addCommands(Commands.runOnce(() -> this.armSubsystem.goTo(null), this.armSubsystem));
    }
}
