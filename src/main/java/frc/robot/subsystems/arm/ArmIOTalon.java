package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.Arm;

public class ArmIOTalon implements ArmIO {

    private final TalonFX armRotLeader;
    private final TalonFX armRotFollower;
    private final DutyCycleEncoder absRotationEncoder;

    private final TalonSRX armExtension;

    private final DigitalInput armTopLimitSwitch;
    private final DigitalInput armBottomLimitSwitch;

    private final DigitalInput extensionTopLimitSwitch;
    private final DigitalInput extensionBottomLimitSwitch;

    public ArmIOTalon() {
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
        armExtension = new WPI_TalonSRX(Constants.Arm.ARM_EXTENSION_LEADER);

        armExtension.configFactoryDefault();

        armExtension.setInverted(true); // TODO: Change if it goes the wrong way.

        absRotationEncoder = new DutyCycleEncoder(Arm.ABS_ROTATION_ENCODER);

        /* Limit Switches */
        armTopLimitSwitch = new DigitalInput(1);
        armBottomLimitSwitch = new DigitalInput(2);

        extensionTopLimitSwitch = new DigitalInput(5);
        extensionBottomLimitSwitch = new DigitalInput(6);

        resetToAbsolute();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armExtension = armExtension.getSelectedSensorPosition();

        inputs.armRot =
                Conversions.falconToDegrees(
                        armRotLeader.getSelectedSensorPosition(), Arm.ROTATION_GEAR_RATIO);
        inputs.absArmRot = absRotationEncoder.getAbsolutePosition();

        inputs.topRotationLimit = armTopLimitSwitch.get();
        inputs.bottomRotationLimit = armTopLimitSwitch.get();
        inputs.minExtensionLimit = extensionBottomLimitSwitch.get();
        inputs.maxExtensionLimit = extensionTopLimitSwitch.get();
    }

    @Override
    public void setArmRotation(double degrees) {
        armRotLeader.set(
                ControlMode.Position,
                Conversions.degreesToFalcon(degrees, Constants.Arm.ROTATION_GEAR_RATIO));
    }

    @Override
    public void setArmExtension(double meters) {

        armExtension.set(
                ControlMode.Position,
                Conversions.degreesToFalcon(
                        (meters / Constants.Arm.EXTENSION_PULLEY_CIRCUMFERENCE * 360),
                        Constants.Arm.EXTENSION_GEAR_RATIO));
    }

    @Override
    public void setArmRotationSpeed(double speed) {
        armRotLeader.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setArmExtensionSpeed(double speed) {
        armExtension.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void checkLimitSwitches() {
        // If velocity 0 return
        // If velocity == Math.abs velocity and top limit switch hit
        // Check arm velocity,

        if (armTopLimitSwitch.get() && armRotLeader.getSelectedSensorVelocity() > 0)
            armRotLeader.set(ControlMode.PercentOutput, 0);
        if (armBottomLimitSwitch.get() && armRotLeader.getSelectedSensorVelocity() < 0)
            armRotLeader.set(ControlMode.PercentOutput, 0);

        if (extensionTopLimitSwitch.get() && armExtension.getSelectedSensorVelocity() > 0)
            armExtension.set(ControlMode.PercentOutput, 0);
        if (extensionBottomLimitSwitch.get() && armExtension.getSelectedSensorVelocity() < 0)
            armExtension.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void resetToAbsolute() {
        armRotLeader.setSelectedSensorPosition(
                Conversions.degreesToFalcon(
                        absRotationEncoder.getAbsolutePosition() - Arm.ARM_ROT_OFFSET,
                        Arm.ROTATION_GEAR_RATIO));
    }
}
