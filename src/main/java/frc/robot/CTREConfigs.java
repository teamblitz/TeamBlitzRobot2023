package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    private static CTREConfigs instance;

    public static CTREConfigs getInstance() {
        if (instance == null) {
            instance = new CTREConfigs();
        }
        return instance;
    }

    public final CANCoderConfiguration swerveCanCoderConfig;

    private CTREConfigs() {
        swerveCanCoderConfig = new CANCoderConfiguration();

        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.CAN_CODER_INVERT;
        swerveCanCoderConfig.initializationStrategy =
                SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
