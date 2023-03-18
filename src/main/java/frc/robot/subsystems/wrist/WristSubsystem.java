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
import frc.robot.commands.wrist.RotateWristToCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Swerve.*;

public class WristSubsystem extends SubsystemBase implements BlitzSubsystem {
    private final WristIO io;

    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private final ArmSubsystem armSubsystem;

    private final ArmFeedforward feedforward;

    private final Logger logger = Logger.getInstance();

    private final ShuffleboardTab tuningTab = Shuffleboard.getTab("DriveTuning");
    private final ShuffleboardLayout wristPidTuning =
            tuningTab.getLayout("wristPid", BuiltInLayouts.kList);

    private final GenericEntry pEntry =
            wristPidTuning.add("p", ANGLE_KP).getEntry("double");
    private final GenericEntry iEntry =
            wristPidTuning.add("i", ANGLE_KI).getEntry("double");
    private final GenericEntry dEntry =
            wristPidTuning.add("d", ANGLE_KD).getEntry("double");
    
    double p = Constants.Wrist.p;
    double i = Constants.Wrist.i;
    double d = Constants.Wrist.d;


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
    }

    public void setRotationSpeed(double speed) {
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
        double rot = relativeRot - armSubsystem.getRotation();
        double clamped =
                MathUtil.clamp(rot, Constants.Wrist.MIN_ROTATION, Constants.Wrist.MAX_ROTATION);
        // Don't do feedforward if we are clamping, so we don't push into ourselves
        io.setRotationSetpoint(
                clamped,
                rot == clamped
                        ? feedforward.calculate(
                                Math.toRadians(relativeRot), Math.toRadians(velocity))
                        : 0);
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

    public CommandBase rotateToCommand(double rotation) {
        return new RotateWristToCommand(this, rotation, 5);
    }
}
