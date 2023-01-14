package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {
    CANSparkMax frontLeft;
    CANSparkMax frontRight;
    CANSparkMax backLeft;
    CANSparkMax backRight;

    public IntakeIOSparkMax() {
        frontLeft =
                new CANSparkMax(
                        Constants.Intake.FRONT_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight =
                new CANSparkMax(
                        Constants.Intake.FRONT_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        backLeft =
                new CANSparkMax(
                        Constants.Intake.BACK_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        backRight =
                new CANSparkMax(
                        Constants.Intake.BACK_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

        frontLeft.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);
        frontRight.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);
        backLeft.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);
        backRight.setSmartCurrentLimit(Constants.Intake.CURRENT_LIMIT);

        frontLeft.setOpenLoopRampRate(.5);
        frontRight.setOpenLoopRampRate(.5);
        backLeft.setOpenLoopRampRate(.5);
        backRight.setOpenLoopRampRate(.5);

        frontRight.follow(frontLeft, true);
        backRight.follow(backLeft, true);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // TODO: Update inputs
    }

    @Override
    public void setFront(double speed) {
        frontLeft.set(speed);
    }

    @Override
    public void setBack(double speed) {
        backLeft.set(speed);
    }
}
