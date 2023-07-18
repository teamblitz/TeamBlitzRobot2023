package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;
import frc.robot.commands.wrist.HoldWristAtPositionCommand;
import frc.robot.commands.wrist.HoldWristAtRelativePositionCommand;
import frc.robot.commands.wrist.RotateWristCommand;
import frc.robot.commands.wrist.RotateWristRelativeCommand;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Here we can probably increase the clearness of some things, for example calling rotation relative
 * to the robot just relative rotation is not exactly clear what it is relative to (in some places
 * it is called robot relative rotation) but this is kinda long, maybe bot relative?
 */
public class WristSubsystem extends SubsystemBase implements BlitzSubsystem {
    private final WristIO io;

    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private final DoubleSupplier armRotSupplier;
    private final DoubleSupplier armLengthSupplier;

    private final ArmFeedforward feedforward;

    private final Logger logger = Logger.getInstance();

    private final ShuffleboardTab tuningTab = Shuffleboard.getTab("DriveTuning");

    double p = Constants.Wrist.p;
    double i = Constants.Wrist.i;
    double d = Constants.Wrist.d;

    private final ShuffleboardLayout wristPidTuning =
            tuningTab.getLayout("wristPid", BuiltInLayouts.kList);

    private final GenericEntry pEntry = wristPidTuning.add("p", p).getEntry("double");
    private final GenericEntry iEntry = wristPidTuning.add("i", i).getEntry("double");
    private final GenericEntry dEntry = wristPidTuning.add("d", d).getEntry("double");

    public double lastGoal;
    public double lastRelativeGoal;

    public WristSubsystem(
            WristIO io, DoubleSupplier armRotSupplier, DoubleSupplier armLengthSupplier) {

        this.armRotSupplier = armRotSupplier;
        this.armLengthSupplier = armLengthSupplier;

        this.io = io;
        // The wrist is basically an arm, so we treat it as such.
        feedforward =
                new ArmFeedforward(Constants.Wrist.ks, Constants.Wrist.kg, Constants.Wrist.kv);

        lastGoal = getRotation();
        lastRelativeGoal = getRelativeRotation();

        // new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> {
        //     lastGoal = getRotation();
        //     lastRelativeGoal = getRelativeRotation();
        // }))
    }

    public void setHoldGoals() {
        lastGoal = getRotation();
        lastRelativeGoal = getRelativeRotation();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        logger.processInputs("wrist", inputs);

        logger.recordOutput("wrist/lastGoal", lastGoal);
        logger.recordOutput("wrist/lastRelativeGoal", lastRelativeGoal);
        logger.recordOutput("wrist/relative", getRelativeRotation());

        boolean pidChanged = false;

        if (pEntry.getDouble(p) != p) {
            pidChanged = true;
            p = pEntry.getDouble(p);
        }
        if (iEntry.getDouble(i) != i) {
            pidChanged = true;
            i = iEntry.getDouble(i);
        }
        if (dEntry.getDouble(d) != d) {
            pidChanged = true;
            d = dEntry.getDouble(d);
        }

        if (pidChanged) {
            io.setPID(p, i, d);
        }

        // TODO: Some point after tulsa, we should just move the closed loop for the wrist to the
        // rio if we are constantly reseting the position
        // Maybe use the quad output of the encoder as well

        io.seedWristPosition(false);
        io.checkLimitSwitches();
    }

    /**
     * Is it safe (will it cause an overextension) if the wrist is moved in the supplied direction
     * Currently only checks for horizontal limit, (as this is what inspectors are looking for) The
     * wrist is most at risk of causing an overextension here, were as the arm will pull itself in
     * fast if it suspects a height overextension.
     *
     * @param direction signum of the direction positive is upward and negative is down
     * @return a boolean stating if moving the wrist in that direction will cause it to overextend
     */
    private boolean safeToMoveWristIn(double direction) {
        return true;
        // The extension of the arm is the length of the arm multiplied by cosine of the angle
        // double extension =
        //         (armLengthSupplier.getAsDouble() *
        // Math.cos(Math.toRadians(armRotSupplier.getAsDouble())))
        //                 - Constants.Arm.ARM_BASE_DISTANCE_FROM_FRAME;

        // if (extension + Constants.Wrist.END_EFFECTOR_LENGTH <
        // Constants.Arm.MAX_EXTENSION_PAST_FRAME) return true; // Currently the wrist moving can't
        // cause an overextenison

        // // If relative rot is pos and direction is pos we are okay, same for the other way around
        // // If they are opposite, that means we will cross over the danger zone
        // return direction * getRelativeRotation() > 0;
    }

    public void setRotationSpeed(double speed) {
        if ((speed > 0 && inputs.topLimit)
                || (speed < 0 && inputs.bottomLimit)
                || !safeToMoveWristIn(Math.signum(speed))) {
            io.setRotationSpeed(0);
            return;
        }
        io.setRotationSpeed(speed);
    }

    /**
     * Update the onboard pid controller based off of the relative angle of the arm
     *
     * @param relativeRot Rotation relative to the robot in degrees
     * @param velocity Velocity in degrees per second
     */
    public void updateRelativeRotation(double relativeRot, double velocity) {
        // Arm Rot + Wrist Rot = Relative Wrist Rot
        // Wrist rot = Relative Wrist Rot - arm rot

        logger.recordOutput("wrist/wanted_rot_relative", relativeRot);

        double rot = relativeRot - armRotSupplier.getAsDouble();

        logger.recordOutput("wrist/wanted_rot", rot);
        logger.recordOutput("wrist/wanted_velocity", velocity);

        double clamped =
                MathUtil.clamp(rot, Constants.Wrist.MIN_ROTATION, Constants.Wrist.MAX_ROTATION);
        double clampedRelativeRot = clamped + armRotSupplier.getAsDouble();

        if (rot != clamped) {
            velocity = 0;
        }

        if ((clamped > Constants.Wrist.MAX_ROTATION - 10 && inputs.topLimit)
                || (clamped < Constants.Wrist.MIN_ROTATION + 10 && inputs.bottomLimit)) {
            io.setRotationSpeed(0);
            return;
        }

        io.setRotationSetpoint(
                clamped,
                feedforward.calculate(
                        Math.toRadians(clampedRelativeRot + Constants.Wrist.CG_OFFSET),
                        Math.toRadians(velocity)));
    }

    public void updateRotation(double wristRot, double velocity) {
        logger.recordOutput("wrist/wanted_rot", wristRot);
        logger.recordOutput("wrist/wanted_velocity", velocity);
        // Arm Rot + Wrist Rot = Relative Wrist Rot
        // Wrist rot = Relative Wrist Rot - arm rot
        double clamped =
                MathUtil.clamp(
                        wristRot, Constants.Wrist.MIN_ROTATION, Constants.Wrist.MAX_ROTATION);
        double relativeRot = clamped + armRotSupplier.getAsDouble();

        if (wristRot != clamped) {
            velocity = 0;
        }

        if (inputs.topLimit && velocity > 0 || inputs.bottomLimit && velocity < 0) {
            io.setRotationSpeed(0);
            logger.recordOutput("wrist/limit_overide", true);
            return;
        }

        logger.recordOutput("wrist/limit_overide", false);

        io.setRotationSetpoint(
                clamped,
                feedforward.calculate(
                        Math.toRadians(relativeRot + Constants.Wrist.CG_OFFSET),
                        Math.toRadians(velocity)));
    }

    public double getRotation() {
        return inputs.rotation;
    }

    public double getRelativeRotation() {
        return armRotSupplier.getAsDouble() + getRotation();
    }

    public double getRotationSpeed() {
        return inputs.rotationSpeed;
    }

    public CommandBase rotateRobotRelativeToCommand(double rotation) {
        return new RotateWristRelativeCommand(this, rotation, 5);
    }

    public CommandBase rotateToCommand(double rotation) {
        return new RotateWristCommand(this, rotation, 5);
    }

    public CommandBase holdAtCommand() {
        return new HoldWristAtPositionCommand(this);
    }

    public CommandBase holdAtRelativeCommand() {
        return new HoldWristAtRelativePositionCommand(this);
    }

    public CommandBase tuckInWristCommand() {
        return rotateToCommand(Constants.Wrist.Position.TUCKED_IN)
                .beforeStarting(() -> logger.recordOutput("wrist/tuck_in_active", true))
                .finallyDo((b) -> logger.recordOutput("wrist/tuck_in_active", false))
                .andThen(holdAtCommand());
    }

    public CommandBase levelWristCommand() {
        return rotateRobotRelativeToCommand(Constants.Wrist.Position.LEVEL)
                .andThen(holdAtRelativeCommand());
    }

    public CommandBase verticalWristCommand() {
        return rotateRobotRelativeToCommand(Constants.Wrist.Position.VERTICAL)
                .andThen(holdAtRelativeCommand());
    }
}
