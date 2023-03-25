package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.BlitzSubsystem;
import frc.robot.Constants;
import frc.robot.commands.wrist.HoldWristAtPositionCommand;
import frc.robot.commands.wrist.HoldWristAtRelativePositionCommand;
import frc.robot.commands.wrist.RotateWristCommand;
import frc.robot.commands.wrist.RotateWristRelativeCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase implements BlitzSubsystem {
    private final WristIO io;

    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private final ArmSubsystem armSubsystem;

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

    public WristSubsystem(WristIO io, ArmSubsystem armSubsystem) {
        this.io = io;
        this.armSubsystem = armSubsystem;
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
        io.seedWristPosition(false);
        io.checkLimitSwitches();

        // if (armSubsystem.shouldWristTuck()) {
        //     tuckInWristCommand().schedule();
        // }
        armSubsystem.setWristPos(getRotation());
    }

    public void setRotationSpeed(double speed) {
        if ((speed > 0 && inputs.topLimit) || (speed < 0 && inputs.bottomLimit)) {
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

        double rot = relativeRot - armSubsystem.getRotation();

        logger.recordOutput("wrist/wanted_rot", rot);
        logger.recordOutput("wrist/wanted_velocity", velocity);

        double clamped =
                MathUtil.clamp(rot, Constants.Wrist.MIN_ROTATION, Constants.Wrist.MAX_ROTATION);
        double clampedRelativeRot = clamped + armSubsystem.getRotation();

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
        double relativeRot = clamped + armSubsystem.getRotation();

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
        return armSubsystem.getRotation() + getRotation();
    }

    public double getRotationSpeed() {
        return inputs.rotationSpeed;
    }

    public CommandBase rotateRelativeToCommand(double rotation) {
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
        return rotateRelativeToCommand(Constants.Wrist.Position.LEVEL)
                .andThen(holdAtRelativeCommand());
    }

    public CommandBase verticalWristCommand() {
        return rotateRelativeToCommand(Constants.Wrist.Position.VERTICAL)
                .andThen(holdAtRelativeCommand());
    }
}
