package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class ArmIOTalonSpark implements ArmIO {

    private final TalonFX armRotLeader;
    private final TalonFX armRotFollower;
    private final DutyCycleEncoder absRotationEncoder;

    private final CANSparkMax wrist;
    private final RelativeEncoder wristEncoder;
    private final DutyCycleEncoder absWristEncoder;

    private final TalonSRX armExtensionLeft;
    private final TalonSRX armExtensionRight;

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    public ArmIOTalonSpark() {
        /* Arm Rotation */
        armRotLeader = new WPI_TalonFX(Constants.Arm.ARM_ROT_LEADER);
        armRotFollower = new WPI_TalonFX(Constants.Arm.ARM_ROT_FOLLOWER);

        armRotLeader.configFactoryDefault();
        armRotFollower.configFactoryDefault();

        armRotLeader.setNeutralMode(NeutralMode.Brake);
        armRotFollower.setNeutralMode(NeutralMode.Brake);

        armRotLeader.setSelectedSensorPosition(
                Conversions.degreesToFalcon(-90, Arm.ROTATION_GEAR_RATIO));

        armRotLeader.configOpenloopRamp(2);
        armRotFollower.configOpenloopRamp(2);

        armRotLeader.setInverted(InvertType.InvertMotorOutput);
        armRotFollower.follow(armRotLeader);
        armRotFollower.setInverted(InvertType.OpposeMaster);

        /* Arm Extension */
        armExtensionLeft = new WPI_TalonSRX(Constants.Arm.ARM_EXTENSION_LEADER);
        armExtensionRight = new WPI_TalonSRX(Constants.Arm.ARM_EXTENSION_FOLLOWER);

        armExtensionLeft.configFactoryDefault();
        armExtensionRight.configFactoryDefault();

        armExtensionLeft.setInverted(true); // TODO: Change if it goes the wrong way.
        armExtensionRight.setInverted(false);

        absRotationEncoder = new DutyCycleEncoder(Arm.ABS_ROTATION_ENCODER);

        /* Wrist Rotation */
        wrist = new CANSparkMax(Constants.Arm.WRIST_ROT_LEADER, MotorType.kBrushless);
        wristEncoder = wrist.getEncoder();

        wrist.restoreFactoryDefaults();

        wrist.setIdleMode(IdleMode.kBrake);


        wristEncoder
                .setPositionConversionFactor(
                        (1 / Arm.WRIST_GEAR_RATIO) // We do 1 over the gear ratio
                                // because 1 rotation of the motor is < 1 rotation of
                                // the wrist
                                * 360);

        absWristEncoder = new DutyCycleEncoder(Arm.ABS_WRIST_ENCODER);


        /* Limit Switches */
        topLimitSwitch = new DigitalInput(1);
        bottomLimitSwitch = new DigitalInput(2);


        seedArmPosition();
        seedWristPosition();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armExtensionL = armExtensionLeft.getSelectedSensorPosition();
        inputs.armExtensionF = armExtensionRight.getSelectedSensorPosition();

        inputs.armRot =
                Conversions.falconToDegrees(
                        armRotLeader.getSelectedSensorPosition(), Arm.ROTATION_GEAR_RATIO);
        inputs.absArmRot = absRotationEncoder.getAbsolutePosition();

        inputs.wristRot = wristEncoder.getPosition();
        inputs.absWristRot = absWristEncoder.getAbsolutePosition();
    }

    @Override
    public void setArmRotation(double degrees) {
        // Get sensor position and use that to determine rotations?
        armRotLeader.set(
                ControlMode.Position,
                Conversions.degreesToFalcon(degrees, Constants.Arm.ROTATION_GEAR_RATIO));
    }

    @Override
    public void setArmExtension(double meters) {

        armExtensionLeft.set(
                ControlMode.Position,
                Conversions.degreesToFalcon(
                        (meters / Constants.Arm.EXTENSION_PULLEY_CIRCUMFERENCE * 360),
                        Constants.Arm.EXTENSION_GEAR_RATIO));
    }

    @Override
    public void setWristRotation(double rot) {
        wrist.getPIDController().setReference(rot, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setArmRotationSpeed(double speed) {
        armRotLeader.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setArmExtensionSpeed(double speed) {
        armExtensionLeft.set(ControlMode.PercentOutput, speed);
        armExtensionRight.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setLeftExtensionSpeed(double speed) {
        armExtensionLeft.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setRightExtensionSpeed(double speed) {
        armExtensionRight.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setWristRotationSpeed(double speed) {
        wrist.set(speed);
    }

    public void checkLimitSwitches() {
        // If velocity 0 return
        // If velocity == Math.abs velocity and top limit switch hit
        // Check arm velocity,
        armRotLeader.getSelectedSensorVelocity();

        if (topLimitSwitch.get() && armRotLeader.getSelectedSensorVelocity() > 0)
            armRotLeader.set(ControlMode.PercentOutput, 0);
        if (bottomLimitSwitch.get() && armRotLeader.getSelectedSensorVelocity() < 0)
            armRotLeader.set(ControlMode.PercentOutput, 0);
    }

    public void seedWristPosition() {
        wristEncoder.setPosition(absWristEncoder.getAbsolutePosition() - Arm.WRIST_ROT_OFFSET);
    }

    public void seedArmPosition() {
        armRotLeader.setSelectedSensorPosition(Conversions.degreesToFalcon(absRotationEncoder.getAbsolutePosition() - Arm.ARM_ROT_OFFSET, Arm.ROTATION_GEAR_RATIO));
    }
}
