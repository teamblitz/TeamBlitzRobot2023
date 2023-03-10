package frc.robot.subsystems.wrist;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class WristIOSpark implements WristIO {

    private final CANSparkMax wrist;
    private final RelativeEncoder wristEncoder;
    private final DutyCycleEncoder absWristEncoder;

    private final DigitalInput wristTopLimitSwitch;
    private final DigitalInput wristBottomLimitSwitch;


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

        absWristEncoder = new DutyCycleEncoder(Arm.ABS_WRIST_ENCODER);

        /* Limit Switches */

        wristTopLimitSwitch = new DigitalInput(3);
        wristBottomLimitSwitch = new DigitalInput(4);

        seedWristPosition();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.rotation = wristEncoder.getPosition();
        inputs.absoluteRotation = absWristEncoder.getAbsolutePosition();
    }


    @Override
    public void setRotation(double rot) {
        wrist.getPIDController().setReference(rot, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setRotationSpeed(double speed) {
        wrist.set(speed);
    }

    public void checkLimitSwitches() {
        // If velocity == Math.abs velocity and top limit switch hit

        if (wristTopLimitSwitch.get() && wristEncoder.getVelocity() > 0)
            wrist.set(0);
        if (wristBottomLimitSwitch.get() && wristEncoder.getVelocity() < 0)
            wrist.set(0);
    }

    public void seedWristPosition() {
        wristEncoder.setPosition(absWristEncoder.getAbsolutePosition() - Arm.WRIST_ROT_OFFSET);
    }
}
