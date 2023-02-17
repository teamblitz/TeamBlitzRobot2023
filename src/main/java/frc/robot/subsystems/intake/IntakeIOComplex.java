package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

/** Hardware io for the complex intake design */
public class IntakeIOComplex implements IntakeIO {
    private final CANSparkMax frontMotor;
    private final CANSparkMax backMotor;

    public IntakeIOComplex() {
        frontMotor =
                new CANSparkMax(
                        Constants.Intake.Complex.FRONT_MOTOR_ID,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        backMotor =
                new CANSparkMax(
                        Constants.Intake.Complex.BACK_MOTOR_ID,
                        CANSparkMaxLowLevel.MotorType.kBrushless);

        frontMotor.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);
        backMotor.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);

        frontMotor.setOpenLoopRampRate(.5);
        backMotor.setOpenLoopRampRate(.5);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // TODO: Update inputs
    }

    @Override
    public void inCube() {
        frontMotor.set(-Constants.Intake.Complex.IN_SPEED);
    }

    @Override
    public void outCube() {
        frontMotor.set(-Constants.Intake.Complex.OUT_SPEED);
    }

    @Override
    public void inCone() {
        frontMotor.set(-Constants.Intake.Complex.IN_SPEED);
        backMotor.set(Constants.Intake.Complex.IN_SPEED);
    }

    @Override
    public void outCone() {
        frontMotor.set(-Constants.Intake.Complex.OUT_SPEED);
        backMotor.set(Constants.Intake.Complex.OUT_SPEED);
    }

    @Override
    public void stop() {
        frontMotor.set(0);
        backMotor.set(0);
    }
}
