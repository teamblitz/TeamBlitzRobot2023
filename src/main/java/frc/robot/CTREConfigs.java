package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

/**
 * Todo, integrate this with constants and swerve module io spark, this class kinda made sense when
 * we copied this with the serve code, but I don't really like that we are using a singleton for it.
 *
 * <p>Unfortunately, although the Config all routine is nice, it doesn't leave much in the way of
 * static constants. We could probably have this all in constants using static {} blocks
 */
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
