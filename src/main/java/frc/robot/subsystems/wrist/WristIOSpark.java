package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.Angles;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import org.littletonrobotics.junction.Logger;

public class WristIOSpark implements WristIO {

    private final CANSparkMax wrist;
    private final RelativeEncoder wristEncoder;
    private final DutyCycleEncoder absWristEncoder;

    private final DigitalInput wristTopLimitSwitch;
    private final DigitalInput wristBottomLimitSwitch;

    private final Logger logger = Logger.getInstance();

    public WristIOSpark() {
        wrist = new CANSparkMax(Constants.Arm.WRIST_ROT_LEADER, MotorType.kBrushless);
        wristEncoder = wrist.getEncoder();

        wrist.restoreFactoryDefaults();

        wrist.setIdleMode(IdleMode.kBrake);

        wristEncoder.setPositionConversionFactor(
                (1 / Arm.WRIST_GEAR_RATIO) // We do 1 over the gear ratio
                        // because 1 rotation of the motor is < 1 rotation of
                        // the wrist
                        * 360);

        wristEncoder.setVelocityConversionFactor(
                (1 / Arm.WRIST_GEAR_RATIO) // We do 1 over the gear ratio
                        // because 1 rotation of the motor is < 1 rotation of
                        // the wrist
                        * 360);

        wrist.getPIDController().setP(Constants.Wrist.p);
        wrist.getPIDController().setI(Constants.Wrist.i);
        wrist.getPIDController().setD(Constants.Wrist.d);

        absWristEncoder = new DutyCycleEncoder(Arm.ABS_WRIST_ENCODER);

        /* Limit Switches */

        wristTopLimitSwitch = new DigitalInput(Arm.TOP_WRIST_LIMIT_SWITCH);
        wristBottomLimitSwitch = new DigitalInput(Arm.BOTTOM_WRIST_LIMIT_SWITCH);

        seedWristPosition();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.rotation = wristEncoder.getPosition();
        inputs.rotationSpeed = wristEncoder.getVelocity();
        inputs.absoluteRotation = Angles.wrapAngle(getAbsolutePosition());
        inputs.absEncoder = Angles.wrapAngle(-absWristEncoder.getAbsolutePosition() * 360);
    }

    @Override
    public void setRotationSetpoint(double rot, double arbFFPercent) {
        wrist.getPIDController()
                .setReference(
                        rot,
                        CANSparkMax.ControlType.kPosition,
                        0,
                        arbFFPercent,
                        SparkMaxPIDController.ArbFFUnits.kPercentOut);
    }

    @Override
    public void setRotationSpeed(double speed) {
        wrist.set(speed);
    }

    public void checkLimitSwitches() {
        // If velocity == Math.abs velocity and top limit switch hit

        if (wristTopLimitSwitch.get() && wristEncoder.getVelocity() > 0) wrist.set(0);
        if (wristBottomLimitSwitch.get() && wristEncoder.getVelocity() < 0) wrist.set(0);
    }

    @Override
    public void seedWristPosition() {
        if (absWristEncoder.isConnected()) {
            wristEncoder.setPosition(getAbsolutePosition());
        } else {
            System.out.printf(
                    "Wrist absolute encoder disconnected, assuming position %s%n",
                    Constants.Wrist.Position.STARTING);
            wristEncoder.setPosition(Constants.Wrist.Position.STARTING);
        }
    }

    @Override
    public void setPID(double p, double i, double d) {
        wrist.getPIDController().setP(p);
        wrist.getPIDController().setI(i);
        wrist.getPIDController().setD(d);
    }

    private double getAbsolutePosition() {
        return Angles.wrapAngle(
                -absWristEncoder.getAbsolutePosition() * 360 - Constants.Wrist.OFFSET);
    }
}
