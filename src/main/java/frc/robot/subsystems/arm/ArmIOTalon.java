package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.math.Angles;
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

        armRotLeader.configOpenloopRamp(2);
        armRotFollower.configOpenloopRamp(2);

        armRotLeader.setInverted(InvertType.None);
        armRotFollower.follow(armRotLeader);
        armRotFollower.setInverted(InvertType.OpposeMaster);

        armRotLeader.config_kP(0, Arm.ROT_P);
        armRotLeader.config_kI(0, Arm.ROT_I);
        armRotLeader.config_kD(0, Arm.ROT_D);

        // We divide by 10 because the function expects it to be per 100ms (dumb ik)
        armRotLeader.configMotionAcceleration(
                Conversions.degreesToFalcon(Arm.ROTATION_ACCELERATION, Arm.ROTATION_GEAR_RATIO)
                        / 10);
        armRotLeader.configMotionCruiseVelocity(
                Conversions.degreesToFalcon(Arm.ROTATION_VELOCITY, Arm.ROTATION_GEAR_RATIO) / 10);

        /* Arm Extension */
        armExtension = new WPI_TalonSRX(Constants.Arm.ARM_EXTENSION_LEADER);

        armExtension.configFactoryDefault();

        armExtension.setSensorPhase(true);

        armExtension.config_kP(0, Arm.EXT_P);
        armExtension.config_kI(0, Arm.EXT_I);
        armExtension.config_kD(0, Arm.EXT_D);

        armExtension.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, 40, 90, 0.2));

        absRotationEncoder = new DutyCycleEncoder(Arm.ABS_ROTATION_ENCODER);

        /* Limit Switches */
        armTopLimitSwitch = new DigitalInput(Arm.TOP_ROTATION_LIMIT_SWITCH);
        armBottomLimitSwitch = new DigitalInput(Arm.BOTTOM_ROTATION_LIMIT_SWITCH);

        extensionTopLimitSwitch = new DigitalInput(Arm.TOP_EXTENSION_LIMIT_SWITCH);
        extensionBottomLimitSwitch = new DigitalInput(Arm.BOTTOM_EXTENSION_LIMIT_SWITCH);

        seedArmPosition();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armExtension = getArmExtension();

        inputs.armRot =
                Conversions.falconToDegrees(
                        armRotLeader.getSelectedSensorPosition(), Arm.ROTATION_GEAR_RATIO);
        inputs.armRotationSpeed =
                Conversions.falconToDegrees(
                                armRotLeader.getSelectedSensorVelocity(), Arm.ROTATION_GEAR_RATIO)
                        / 10.0;
        inputs.absArmRot = getAbsolutePosition();
        inputs.absArmEncoder = Angles.wrapAngle180(-absRotationEncoder.getAbsolutePosition() * 360);

        inputs.topRotationLimit = armTopLimitSwitch.get();
        inputs.bottomRotationLimit = armBottomLimitSwitch.get();
        inputs.minExtensionLimit = extensionBottomLimitSwitch.get();
        inputs.maxExtensionLimit = extensionTopLimitSwitch.get();

        inputs.encoderConnected = absRotationEncoder.isConnected();
    }

    /** Updates the arm position setpoint. */
    @Override
    public void setRotationSetpoint(double degrees, double arbFFPercent) {
        armRotLeader.set(
                ControlMode.Position,
                Conversions.degreesToFalcon(degrees, Constants.Arm.ROTATION_GEAR_RATIO),
                DemandType.ArbitraryFeedForward,
                arbFFPercent);
    }

    @Override
    public void setExtensionSetpoint(double meters) {

        armExtension.set(
                ControlMode.Position,
                Conversions.degreesToFalcon(
                        (meters / Constants.Arm.EXTENSION_PULLEY_CIRCUMFERENCE * 360),
                        Constants.Arm.EXTENSION_GEAR_RATIO));
    }

    @Override
    public void setArmRotationSpeed(double percent) {
        armRotLeader.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void setArmExtensionSpeed(double percent) {
        armExtension.set(ControlMode.PercentOutput, percent);
    }

    private double getArmExtension() {
        return Conversions.redlineToDegrees(
                        armExtension.getSelectedSensorPosition(), Arm.EXTENSION_GEAR_RATIO)
                / 360
                * Constants.Arm.EXTENSION_PULLEY_CIRCUMFERENCE;
    }

    @Override
    public void checkLimitSwitches() {
        // If velocity 0 return
        // If velocity == Math.abs velocity and top limit switch hit
        // Check arm velocity,

        //        if (armTopLimitSwitch.get() && armRotLeader.getSelectedSensorVelocity() > 0)
        //            armRotLeader.set(ControlMode.PercentOutput, 0);
        //        if (armBottomLimitSwitch.get() && armRotLeader.getSelectedSensorVelocity() < 0)
        //            armRotLeader.set(ControlMode.PercentOutput, 0);

        if (extensionBottomLimitSwitch.get() && armExtension.getSelectedSensorVelocity() < 0)
            armExtension.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void seedArmPosition() {
        if (absRotationEncoder.isConnected()) {
            armRotLeader.setSelectedSensorPosition(
                    Conversions.degreesToFalcon(getAbsolutePosition(), Arm.ROTATION_GEAR_RATIO));
        } else {
            System.out.printf(
                    "Arm absolute rotation encoder disconnected, assuming position %s%n",
                    Arm.STARTING_ROTATION);
            armRotLeader.setSelectedSensorPosition(
                    Conversions.degreesToFalcon(Arm.STARTING_ROTATION, Arm.ROTATION_GEAR_RATIO));
        }
    }

    private double getAbsolutePosition() {
        return Angles.wrapAngle180(
                Angles.wrapAngle180(
                        -absRotationEncoder.getAbsolutePosition() * 360 - Arm.ARM_ROT_OFFSET));
    }
}
