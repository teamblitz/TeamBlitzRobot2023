package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.BlitzSubsystem;
import frc.lib.MutableReference;
import frc.robot.Constants;
import frc.robot.commands.arm.ExtendToCommand;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Maybe divide this into 2 subsystems, depends on how we want to control it. The current way we do
 * this, 2 subsystems is ideal (and is kinda what we are pseudo doing)
 */
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
    public final Subsystem extensionRequirement = new Subsystem() {};
    public final Subsystem rotationRequirement = new Subsystem() {};

    public final DoubleSupplier relativeWristRotSupplier;
    public final DoubleSupplier wristRotSupplier;

    private final CommandBase protectArmCommand;
    private boolean stopExtendingOut;

    private boolean shouldTuck;
    private double extension;

    public ArmSubsystem(
            ArmIO io, DoubleSupplier relativeWristRotSupplier, DoubleSupplier wristRotSupplier) {
        this.io = io;

        // Prevent commands from requiring this subsystem, instead use the ExtensionRequirement and
        // Rotation Requirement subsystems
        Commands.run(() -> {})
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                .schedule();

        new Trigger(() -> inputs.encoderConnected).onTrue(
                runOnce(io::seedArmPosition)
                        .andThen(() -> io.setArmRotationSpeed(0)) // Set the arm to 0 to end on board pid loop
                        .ignoringDisable(true)
                        .));



        protectArmCommand = extendToCommand(Constants.Arm.PULL_TO);

        this.relativeWristRotSupplier = relativeWristRotSupplier;
        this.wristRotSupplier = wristRotSupplier;
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

        // All the below is backup
        // I don't want to delete the one thing that got us though inspection in denver
        // Before we know if it works or not

        // Extension limit
        //        extension =
        //                (armLength * Math.cos(Math.toRadians(getRotation())))
        //                        - Constants.Arm.ARM_BASE_DISTANCE_FROM_FRAME;
        //        logger.recordOutput("arm/extension", extension);
        //
        //        double max =
        //                Constants.Arm.STOP_EXTENSION + (wristPos > -160 ?
        // Units.inchesToMeters(-10) : 0);
        //
        //        if (extension > max) {
        //            stopExtendingOut = true;
        //            logger.recordOutput("arm/extension_protection_enabled", true);
        //        } else {
        //            stopExtendingOut = false;
        //            logger.recordOutput("arm/extension_protection_enabled", false);
        //        }
    }

    // Pretty much the same impl as wrist
    private boolean safeToRotateArmIn(double direction) {
        return true;
        // double extension =
        //         (getExtension() * Math.cos(Math.toRadians(getRotation())))
        //                 - Constants.Arm.ARM_BASE_DISTANCE_FROM_FRAME
        //         + Math.max(0, Constants.Wrist.END_EFFECTOR_LENGTH *
        // Math.cos(Math.toRadians(relativeWristRotSupplier.getAsDouble())));

        // return extension < Constants.Arm.MAX_EXTENSION_PAST_FRAME || direction * getRotation() >
        // 0;
    }

    // Very similar impl to above
    private boolean safeToExtendArmIn(double direction) {
        double extension =
                (getExtension() * Math.cos(Math.toRadians(getRotation())))
                        - Constants.Arm.ARM_BASE_DISTANCE_FROM_FRAME;
        // + Math.max(0, Constants.Wrist.END_EFFECTOR_LENGTH *
        // Math.cos(Math.toRadians(relativeWristRotSupplier.getAsDouble())));

        extension += (wristRotSupplier.getAsDouble() < -130) ? 0 : Units.inchesToMeters(10);

        return extension < Constants.Arm.MAX_EXTENSION_PAST_FRAME || direction < 0;
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
        logger.recordOutput("arm/wanted_extension", meters);
        io.setExtensionSetpoint(meters);
    }

    public double getRotation() {
        return inputs.armRot;
    }

    public double getExtension() {
        return inputs.armExtension;
    }

    /**
     * Returns the "real world" distance in between the arm and wrist center of rotations, used for
     * anti over extenison.
     *
     * @return ArmLength in meters
     */
    public double getArmLength() {

        double percentExtended = getExtension() / Constants.Arm.MAX_EXTENSION;

        return (Constants.Arm.OUT_LENGTH - Constants.Arm.IN_LENGTH) * percentExtended
                + Constants.Arm.IN_LENGTH;
    }

    public double getRotationSpeed() {
        return inputs.armRotationSpeed;
    }

    public double getExtensionSpeed() {
        return inputs.armExtensionSpeed;
    }

    public void setArmRotationSpeed(double percent) {
        if (!safeToRotateArmIn(Math.signum(percent))) {
            io.setArmRotationSpeed(0);
            return;
        }
        io.setArmRotationSpeed(percent);
    }

    public void setArmExtensionSpeed(double percent) {
        // if (stopExtendingOut && percent > 0) {
        //     io.setArmExtensionSpeed(0);
        //     return;
        // }
        // if (extension > Constants.Arm.SLOW_DOWN_AT) {
        //     io.setArmExtensionSpeed(percent * .5);

        // }
        if (!safeToExtendArmIn(Math.signum(percent))) {
            io.setArmExtensionSpeed(0);
            return;
        }
        io.setArmExtensionSpeed(percent);
    }

    public CommandBase rotateToCommand(double degrees) {
        double goal = MathUtil.clamp(degrees, Constants.Arm.MIN_ROT, Constants.Arm.MAX_ROT);

        MutableReference<TrapezoidProfile> profile = new MutableReference<>();
        Timer timer = new Timer();

        return runOnce(
                        () -> {
                            profile.set(
                                    new TrapezoidProfile(
                                            new TrapezoidProfile.Constraints(
                                                    Constants.Arm.ROTATION_VELOCITY,
                                                    Constants.Arm.ROTATION_ACCELERATION),
                                            new TrapezoidProfile.State(goal, 0),
                                            new TrapezoidProfile.State(
                                                    getRotation(), getRotationSpeed())));
                            timer.restart();
                        })
                .andThen(
                        run(
                                () -> {
                                    TrapezoidProfile.State setpoint =
                                            profile.get().calculate(timer.get());
                                    updateRotation(setpoint.position, setpoint.velocity);
                                }))
                .until(() -> profile.get().isFinished(timer.get()))
                .finallyDo(
                        (interrupted) ->
                                updateRotation(profile.get().calculate(timer.get()).position, 0))
                .
    }

    public CommandBase extendToCommand(double meters) {
        return new ExtendToCommand(
                        this,
                        MathUtil.clamp(
                                meters,
                                Constants.Arm.MIN_EXTENSION + Units.inchesToMeters(0.5),
                                Constants.Arm.MAX_EXTENSION),
                        Constants.Arm.EXTENSION_THRESHOLD)
                .withTimeout(5);
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

}
