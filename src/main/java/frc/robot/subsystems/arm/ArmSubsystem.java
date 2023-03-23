package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.BlitzSubsystem;
import frc.lib.math.controller.TelescopingArmFeedforward;
import frc.robot.Constants;
import frc.robot.commands.arm.ExtendToCommand;
import frc.robot.commands.arm.RotateToCommand;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements BlitzSubsystem {

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private final Logger logger = Logger.getInstance();

    // Because of how command scheduler requirements work, 2 commands can't use the same subsystem
    // at the same time,
    // While this is normally a good thing, we want the 2 mechanisms to work in parallel from 2
    // separate commands at times.
    // This could be avoided by creating 2 subsystems, or more closely meshing the control of both
    // extension and rotation,
    // But my 12:30am brain thinks this is the best way to do it atm.
    // This assumes that commands requiring extension requirement will only command the extension
    // and vice versa.
    public final Subsystem ExtensionRequirement = new Subsystem() {};
    public final Subsystem RotationRequirement = new Subsystem() {};

    private final TelescopingArmFeedforward rotationFeedforward;

    private final CommandBase protectArmCommand;
    private boolean stopExtendingOut;

    private boolean shouldTuck;

    public ArmSubsystem(ArmIO io) {
        this.io = io;

        rotationFeedforward =
                new TelescopingArmFeedforward(
                        (x) -> 0., (x) -> 0., (x) -> 0.,
                        (x) -> 0.); // TODO: Make these actual gains

        // Prevent commands from requiring this subsystem, instead use the ExtensionRequirement and
        // Rotation Requirement subsystems
        Commands.run(() -> {}).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);

        // Schedule a command to seed the arm, as the encoder does not appear to be connected when
        // this class is initiated.
        Commands.waitSeconds(5).andThen(this::seedArm).ignoringDisable(true).schedule();

        protectArmCommand = extendToCommand(Constants.Arm.PULL_TO);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        logger.processInputs("arm", inputs);

        // Height Protection
        double percentExtended = getExtension() / Constants.Arm.MAX_EXTENSION;
        double armLength =
                (Constants.Arm.OUT_LENGTH - Constants.Arm.IN_LENGTH) * percentExtended
                        + Constants.Arm.IN_LENGTH;
        double height =
                (Math.sin(Math.toRadians(getRotation())) * armLength)
                        + Constants.Arm.ARM_BASE_HEIGHT;

        if (height > Constants.Arm.HEIGHT_PROTECTION) {
            protectArmCommand.schedule();
            logger.recordOutput("arm/height_protection_enabled", true);
        } else {
            protectArmCommand.cancel();
            logger.recordOutput("arm/height_protection_enabled", false);
        }
        logger.recordOutput("arm/length", armLength);
        // Extension limit

        double extension =
                (armLength * Math.cos(Math.toRadians(getRotation())))
                        - Constants.Arm.ARM_BASE_DISTANCE_FROM_FRAME;
        logger.recordOutput("arm/extension", extension);

        if (extension > Constants.Arm.TUCK_IN_EXTENSION) {
            shouldTuck = true;
        } else {
            shouldTuck = false;
        }

        if (extension > Constants.Arm.STOP_EXTENSION) {
            stopExtendingOut = true;
            logger.recordOutput("arm/extension_protection_enabled", true);
        } else {
            stopExtendingOut = false;
            logger.recordOutput("arm/extension_protection_enabled", false);
        }
    }

    public boolean shouldWristTuck() {
        return shouldTuck;
    }

    public void updateRotation(double degrees, double velocity) {
        logger.recordOutput("arm/wanted_rotation", degrees);
        io.setRotationSetpoint(
                degrees,
                0); // Don't do feed forward as we have no way to model this with the spring-loaded
        // arm
    }

    // Currently velocity is unused as we do not do feed forward on the telescoping aspect of the
    // arm
    public void updateExtension(double meters, double velocity) {

        if (inputs.minExtensionLimit && meters < .01) setArmExtensionSpeed(0);
        io.setExtensionSetpoint(meters);
    }

    public double getRotation() {
        return inputs.armRot;
    }

    public double getExtension() {
        return inputs.armExtension;
    }

    public double getRotationSpeed() {
        return inputs.armRotationSpeed;
    }

    public double getExtensionSpeed() {
        return inputs.armExtensionSpeed;
    }

    public void setArmRotationSpeed(double percent) {
        io.setArmRotationSpeed(percent);
    }

    public void setArmExtensionSpeed(double percent) {
        if (stopExtendingOut && percent > 0) {
            io.setArmExtensionSpeed(0);
            return;
        }
        io.setArmExtensionSpeed(percent);
    }

    public CommandBase rotateToCommand(double degrees) {
        return new RotateToCommand(
                this,
                MathUtil.clamp(degrees, Constants.Arm.MIN_ROT, Constants.Arm.MAX_ROT),
                Constants.Arm.ROT_THRESHOLD);
    }

    public CommandBase extendToCommand(double meters) {
        return new ExtendToCommand(
                this,
                MathUtil.clamp(meters, Constants.Arm.MIN_EXTENSION, Constants.Arm.MAX_EXTENSION),
                Constants.Arm.EXTENSION_THRESHOLD);
    }

    public CommandBase retractArmCommand() {
        return extendToCommand(Constants.Arm.Position.Extension.RETRACTED);
    }

    public CommandBase homeArmCommand() {
        return retractArmCommand()
                .andThen(rotateToCommand(Constants.Arm.Position.Rotation.VERTICAL));
    }

    public CommandBase levelArmCommand() {
        return rotateToCommand(Constants.Arm.Position.Rotation.LEVEL);
    }

    public void seedArm() {
        io.seedArmPosition();
    }
}
