package frc.robot.commands;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Leds.LedSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class CommandBuilder {
    private final  ArmSubsystem armSubsystem;
    private final  DriveSubsystem driveSubsystem;
    private final  IntakeSubsystem intakeSubsystem;
    private final  LedSubsystem ledSubsystem;
    private final  WristSubsystem wristSubsystem;

    public CommandBuilder(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, LedSubsystem ledSubsystem, WristSubsystem wristSubsystem) {
        this.armSubsystem = armSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.wristSubsystem = wristSubsystem;
    }

    /* Arm/Wrist/Manipulator Commands */
    /**
     * Prime the arm by tucking in the wrist (to prevent over extending), lower down the arm, and extend to needed distance, and finally positioning the arm and wrist
     */
    public CommandBase primeConeHighCommand() {
        return wristSubsystem.tuckInWristCommand().alongWith(armSubsystem.levelArmCommand())
                .andThen(armSubsystem.extendToCommand(Constants.Arm.Position.Extension.CONE_HIGH))
                .andThen(armSubsystem.rotateToCommand(Constants.Arm.Position.Rotation.CONE_HIGH)
                        .alongWith(Commands.waitUntil(() -> armSubsystem.getRotation() > Constants.Arm.Position.Rotation.CONE_HIGH / 2)
                                .andThen(wristSubsystem.rotateToCommand(Constants.Wrist.Position.CONE_HIGH_RELATIVE - Constants.Arm.Position.Rotation.CONE_HIGH)))
                );
    }
}
