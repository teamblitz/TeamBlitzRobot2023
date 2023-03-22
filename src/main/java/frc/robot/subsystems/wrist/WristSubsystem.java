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

    public WristSubsystem(WristIO io, ArmSubsystem armSubsystem) {
        this.io = io;
        this.armSubsystem = armSubsystem;
        // The wrist is basically an arm, so we treat it as such.
        feedforward =
                new ArmFeedforward(Constants.Wrist.ks, Constants.Wrist.kg, Constants.Wrist.kv);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        logger.processInputs("wrist", inputs);

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
        io.seedWristPosition();
    }

    public void setRotationSpeed(double speed) {
        io.setRotationSpeed(speed);
    }

    public void openLoopGoTo(double degrees) {
        double relativeRot = degrees + armSubsystem.getRotation();
        logger.recordOutput("wrist/goToRelative", relativeRot);
        io.setVoltage(feedforward.calculate(Math.toRadians(relativeRot), 0));
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
        double rot = relativeRot - armSubsystem.getRotation();

        logger.recordOutput("wrist/wanted_rot", rot);
        logger.recordOutput("wrist/wanted_velocity", velocity);

        double clamped =
                MathUtil.clamp(rot, Constants.Wrist.MIN_ROTATION, Constants.Wrist.MAX_ROTATION);
        // Don't do feedforward if we are clamping, so we don't push into ourselves
        io.setRotationSetpoint(
                clamped,
                rot == clamped && getRotation() < -10
                        ? feedforward.calculate(
                                Math.toRadians(relativeRot + Constants.Wrist.CG_OFFSET),
                                Math.toRadians(velocity))
                        : 0);
    }

    public void updateRotation(double wristRot, double velocity) {
        logger.recordOutput("wrist/wanted_rot", wristRot);
        logger.recordOutput("wrist/wanted_velocity", velocity);
        // Arm Rot + Wrist Rot = Relative Wrist Rot
        // Wrist rot = Relative Wrist Rot - arm rot
        double relativeRot = wristRot + armSubsystem.getRotation();
        double clamped =
                MathUtil.clamp(
                        relativeRot, Constants.Wrist.MIN_ROTATION, Constants.Wrist.MAX_ROTATION);
        double ff =
                feedforward.calculate(
                        Math.toRadians(relativeRot + Constants.Wrist.CG_OFFSET),
                        Math.toRadians(velocity));
        // Don't do feedforward if we are clamping, so we don't push into ourselves
        io.setRotationSetpoint(
                clamped, relativeRot == clamped && getRotation() < -10 && velocity > 0 ? ff : 0);
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
        return rotateToCommand(Constants.Wrist.Position.TUCKED_IN).andThen(holdAtCommand());
    }

    public CommandBase levelWristCommand() {
        return rotateRelativeToCommand(Constants.Wrist.Position.LEVEL)
                .andThen(holdAtRelativeCommand());
    }

    public CommandBase verticalWristCommand() {
        return rotateRelativeToCommand(Constants.Wrist.Position.VERTICAL)
                .andThen(holdAtRelativeCommand());
    }

    public void seedWrist() {
        io.seedWristPosition();
    }
}
