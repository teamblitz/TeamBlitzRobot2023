package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ArmIOTalonSpark implements ArmIO {

    private final CANSparkMax armRotLeader;
    private final CANSparkMax armRotFollower;

    private final CANSparkMax wristRotLeader;
    private final CANSparkMax wristRotFollower;

    private final TalonSRX armExtensionLeader;
    private final TalonSRX armExtensionFollower;

    public ArmIOTalonSpark() {
        armRotLeader = new CANSparkMax(Constants.Arm.ARM_ROT_LEADER, MotorType.kBrushless);
        armRotFollower = new CANSparkMax(Constants.Arm.ARM_ROT_FOLLOWER, MotorType.kBrushless);

        wristRotLeader = new CANSparkMax(Constants.Arm.WRIST_ROT_LEADER, MotorType.kBrushless);
        wristRotFollower = new CANSparkMax(Constants.Arm.WRIST_ROT_FOLLOWER, MotorType.kBrushless);

        armExtensionLeader = new WPI_TalonSRX(Constants.Arm.ARM_EXTENSION_LEADER);
        armExtensionFollower = new WPI_TalonSRX(Constants.Arm.ARM_EXTENSION_FOLLOWER);
    }

    public void setArmRotation(Rotation2d rot) {}

    public void setArmPosition(double meters) {}

    public void setWristRotation(Rotation2d rot) {}

    public void setArmRotationSpeed(double speed) {}

    public void setArmExtensionSpeed(double speed) {}

    public void setWristRotationSpeed(double speed) {}
}
