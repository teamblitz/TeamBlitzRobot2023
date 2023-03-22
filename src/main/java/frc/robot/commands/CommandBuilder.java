package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class CommandBuilder {
    private final ArmSubsystem armSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LedSubsystem ledSubsystem;
    private final WristSubsystem wristSubsystem;

    public CommandBuilder(
            ArmSubsystem armSubsystem,
            DriveSubsystem driveSubsystem,
            IntakeSubsystem intakeSubsystem,
            LedSubsystem ledSubsystem,
            WristSubsystem wristSubsystem) {
        this.armSubsystem = armSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.wristSubsystem = wristSubsystem;
    }

    /* Arm/Wrist/Manipulator Commands */
    public CommandBase levelAndTuck(boolean tuckIn) {
        return tuckIn
                ? armSubsystem.levelArmCommand().alongWith(wristSubsystem.tuckInWristCommand())
                : armSubsystem.levelArmCommand();
    }

    /**
     * Prime the arm by tucking in the wrist (to prevent over extending), lower down the arm, and
     * extend to needed distance, and finally positioning the arm and wrist.
     *
     * @param rotation end rotation
     * @param extension end extension
     * @param wristRot final relative rotation of the wrist
     * @param tuckIn should we tuck in to avoid overextension
     * @return composition of the commands
     */
    public CommandBase primeFor(
            double rotation, double extension, double wristRot, boolean tuckIn) {
        return levelAndTuck(tuckIn)
                .andThen(armSubsystem.extendToCommand(extension))
                .andThen(
                        armSubsystem
                                .rotateToCommand(rotation)
                                .alongWith(
                                        Commands.waitUntil(
                                                        () ->
                                                                armSubsystem.getRotation()
                                                                        > rotation / 2)
                                                .andThen(
                                                        wristSubsystem.rotateToCommand(
                                                                wristRot - rotation))));
    }

    public CommandBase primeConeHigh() {
        return primeFor(
                Constants.Arm.Position.Rotation.CONE_HIGH,
                Constants.Arm.Position.Extension.CONE_HIGH,
                Constants.Wrist.Position.CONE_HIGH_RELATIVE,
                true);
    }

    public CommandBase primeConeMid() {
        return primeFor(
                Constants.Arm.Position.Rotation.CONE_MID,
                Constants.Arm.Position.Extension.CONE_MID,
                Constants.Wrist.Position.CONE_MID_RELATIVE,
                false);
    }

    public CommandBase primeCubeHigh() {
        return primeFor(
                Constants.Arm.Position.Rotation.CUBE_HIGH,
                Constants.Arm.Position.Extension.CUBE_HIGH,
                Constants.Wrist.Position.CUBE_HIGH_RELATIVE,
                true);
    }

    public CommandBase primeCubeMid() {
        return primeFor(
                Constants.Arm.Position.Rotation.CUBE_MID,
                Constants.Arm.Position.Extension.CUBE_MID,
                Constants.Wrist.Position.CUBE_MID_RELATIVE,
                false);
    }

    public CommandBase primeHybrid() {
        return armSubsystem
                .rotateToCommand(Constants.Arm.Position.Rotation.HYBRID)
                .alongWith(wristSubsystem.rotateToCommand(-5));
    }

    public CommandBase primeCubeRamp() {
        return armSubsystem
                .rotateToCommand(Constants.Arm.Position.Rotation.RAMP)
                .alongWith(wristSubsystem.rotateToCommand(Constants.Wrist.Position.RAMP));
    }

    public CommandBase primeCubeShelf() {
        return primeFor(
                Constants.Arm.Position.Rotation.CUBE_SHELF,
                Constants.Arm.Position.Extension.CUBE_SHELF,
                Constants.Wrist.Position.CUBE_SHELF_RELATIVE,
                true);
    }

    public CommandBase primeConeShelf() {
        return primeFor(
                Constants.Arm.Position.Rotation.CONE_SHELF,
                Constants.Arm.Position.Extension.CONE_SHELF,
                Constants.Wrist.Position.CONE_SHELF_RELATIVE,
                true);
    }
}
