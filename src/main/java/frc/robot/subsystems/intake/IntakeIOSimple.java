package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

/** Hardware io for the simple intake design */
public class IntakeIOSimple implements IntakeIO {

    private final CANSparkMax motor;

    public IntakeIOSimple() {
        motor =
                new CANSparkMax(
                        Constants.Intake.Simple.MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);

        motor.setOpenLoopRampRate(.5);

        motor.setInverted(true);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rpm = motor.getEncoder().getVelocity();
        inputs.current = motor.getOutputCurrent();
    }

    @Override
    public void inCone() {
        in();
    }

    @Override
    public void outCone() {
        out();
    }

    @Override
    public void inCube() {
        in();
    }

    @Override
    public void outCube() {
        out();
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    @Override
    public void set(double percent) {
        motor.set(percent);
    }

    private void in() {
        motor.set(Constants.Intake.Simple.IN_SPEED);
    }

    private void out() {
        motor.set(Constants.Intake.Simple.OUT_SPEED);
    }
}
