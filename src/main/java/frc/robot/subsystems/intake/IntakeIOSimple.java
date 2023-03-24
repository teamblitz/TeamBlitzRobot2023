package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

/** Hardware io for the simple intake design */
public class IntakeIOSimple implements IntakeIO {

    private final CANSparkMax leaderMotor;
    private final CANSparkMax followerMotor;

    public IntakeIOSimple() {
        leaderMotor =
                new CANSparkMax(
                        Constants.Intake.Simple.LEFT_MOTOR_ID,
                        CANSparkMaxLowLevel.MotorType.kBrushless);
        followerMotor =
                new CANSparkMax(
                        Constants.Intake.Simple.RIGHT_MOTOR_ID,
                        CANSparkMaxLowLevel.MotorType.kBrushless);

        leaderMotor.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);
        followerMotor.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);

        leaderMotor.setOpenLoopRampRate(.5);
        followerMotor.setOpenLoopRampRate(.5);

        leaderMotor.setInverted(true);

        followerMotor.follow(leaderMotor, true);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // TODO: Update inputs
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
        leaderMotor.set(0);
    }

    private void in() {
        leaderMotor.set(Constants.Intake.Simple.IN_SPEED);
    }

    private void out() {
        leaderMotor.set(Constants.Intake.Simple.OUT_SPEED);
    }
}
