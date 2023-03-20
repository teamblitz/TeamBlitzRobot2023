package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private final TelescopingArmFeedforward rotationFeedforward;

    public ArmSubsystem(ArmIO io) {
        this.io = io;

        rotationFeedforward =
                new TelescopingArmFeedforward(
                        (x) -> 0., (x) -> 0., (x) -> 0.,
                        (x) -> 0.); // TODO: Make these actual gains
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        logger.processInputs("arm", inputs);
    }

    public void updateRotation(double degrees, double velocity) {
        io.setRotationSetpoint(
                degrees,
                0); // Don't do feed forward as we have no way to model this with the spring-loaded
        // arm
    }

    // Currently velocity is unused as we do not do feed forward on the telescoping aspect of the
    // arm
    public void updateExtension(double meters, double velocity) {
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
        return extendToCommand(Constants.Arm.Position.RETRACTED);
    }

    public CommandBase homeArmCommand() {
        return retractArmCommand().andThen(rotateToCommand(Constants.Arm.Position.VERTICAL));
    }
}
