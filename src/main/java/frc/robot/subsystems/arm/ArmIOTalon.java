package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.Conversions;
import frc.lib.math.controller.TelescopingArmFeedforward;
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

        // We divide by 10 because the function expects it to be per 100ms (dumb ik)
        armRotLeader.configMotionAcceleration(
                Conversions.degreesToFalcon(
                                Arm.ACCELERATION_METERS_PER_SECOND_SQUARED, Arm.ROTATION_GEAR_RATIO)
                        / 10);
        armRotLeader.configMotionCruiseVelocity(
                Conversions.degreesToFalcon(Arm.VELOCITY_METERS_PER_SECOND, Arm.ROTATION_GEAR_RATIO)
                        / 10);


        /* Arm Extension */
        armExtension = new WPI_TalonSRX(Constants.Arm.ARM_EXTENSION_LEADER);

        armExtension.configFactoryDefault();

        armExtension.setInverted(true); // TODO: Change if it goes the wrong way.

        absRotationEncoder = new DutyCycleEncoder(Arm.ABS_ROTATION_ENCODER);

        /* Limit Switches */
        armTopLimitSwitch = new DigitalInput(Arm.TOP_ROTATION_LIMIT_SWITCH);
        armBottomLimitSwitch = new DigitalInput(Arm.BOTTOM_ROTATION_LIMIT_SWITCH);

        extensionTopLimitSwitch = new DigitalInput(Arm.TOP_EXTENSTION_LIMIT_SWITCH);
        extensionBottomLimitSwitch = new DigitalInput(Arm.BOTTOM_EXTENSION_LIMIT_SWITCH);

        resetToAbsolute();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armExtension = getArmExtension();

        inputs.armRot =
                Conversions.falconToDegrees(
                        armRotLeader.getSelectedSensorPosition(), Arm.ROTATION_GEAR_RATIO);
        inputs.absArmRot = absRotationEncoder.getAbsolutePosition();

        inputs.topRotationLimit = armTopLimitSwitch.get();
        inputs.bottomRotationLimit = armTopLimitSwitch.get();
        inputs.minExtensionLimit = extensionBottomLimitSwitch.get();
        inputs.maxExtensionLimit = extensionTopLimitSwitch.get();
    }

    /**
     * Updates the arm position setpoint.
     */
    public void setRotationSetpoint(double degrees, double arbFFPercent) {
        armRotLeader.set(
                ControlMode.Position,
                Conversions.degreesToFalcon(degrees, Constants.Arm.ROTATION_GEAR_RATIO),
                DemandType.ArbitraryFeedForward,
                arbFFPercent);
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

    private double getArmExtension() {
        return Conversions.redlineToDegrees(armExtension.getSelectedSensorPosition(), Arm.EXTENSION_GEAR_RATIO);
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
