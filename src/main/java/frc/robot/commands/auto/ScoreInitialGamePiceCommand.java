package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ScoreInitialGamePiceCommand extends SequentialCommandGroup {
    private final ArmSubsystem armSubsystem;
    private final IntakeSubsystem intakeSubsystem;


    public ScoreInitialGamePiceCommand(final ArmSubsystem armSubsystem, final IntakeSubsystem intakeSubsystem) {
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addCommands(Commands.runOnce(() -> armSubsystem.goTo(null), armSubsystem));

intakeSubsystem.buildConeInCommand(), intakeSubsystem.buildConeOutCommand(), intakeSubsystem.buildCubeInCommand(), intakeSubsystem.buildCubeOutCommand());
    }
}
