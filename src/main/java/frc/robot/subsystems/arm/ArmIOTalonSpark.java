package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ArmIOTalonSpark implements ArmIO {

    private final TalonFX armRotLeader;
    private final TalonFX armRotFollower;

    private final CANSparkMax wristRotLeader;
    private final CANSparkMax wristRotFollower;

    private final TalonSRX armExtensionLeader;
    private final TalonSRX armExtensionFollower;

    public ArmIOTalonSpark() {
        armRotLeader = new WPI_TalonFX(Constants.Arm.ARM_ROT_LEADER);
        armRotFollower = new WPI_TalonFX(Constants.Arm.ARM_ROT_FOLLOWER);

        wristRotLeader = new CANSparkMax(Constants.Arm.WRIST_ROT_LEADER, MotorType.kBrushless);
        wristRotFollower = new CANSparkMax(Constants.Arm.WRIST_ROT_FOLLOWER, MotorType.kBrushless);

        armExtensionLeader = new WPI_TalonSRX(Constants.Arm.ARM_EXTENSION_LEADER);
        armExtensionFollower = new WPI_TalonSRX(Constants.Arm.ARM_EXTENSION_FOLLOWER);

        armRotLeader.configFactoryDefault();
        armRotFollower.configFactoryDefault();

        armRotFollower.follow(armRotLeader);
        armRotFollower.setInverted(InvertType.OpposeMaster);

        wristRotLeader.restoreFactoryDefaults();
        wristRotFollower.restoreFactoryDefaults();

        wristRotFollower.follow(wristRotLeader, true);

        armExtensionLeader.configFactoryDefault();
        armExtensionFollower.configFactoryDefault();

        armExtensionFollower.follow(armExtensionLeader);
        armExtensionFollower.setInverted(InvertType.OpposeMaster);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // TODO: Update inputs
    }

    @Override
    public void setArmRotation(Rotation2d rot) {
        // Get sensor position and use that to determine rotations?
    }

    @Override
    public void setArmExtension(double meters) {
    }

    @Override
    public void setWristRotation(Rotation2d rot) {

    }

    @Override
    public void setArmRotationSpeed(double speed) {
        armRotLeader.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setArmExtensionSpeed(double speed) {
        armExtensionLeader.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void setWristRotationSpeed(double speed) {
        wristRotLeader.set(speed);
    }
}
