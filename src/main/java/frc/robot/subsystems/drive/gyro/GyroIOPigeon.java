package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.Constants;

public class GyroIOPigeon implements GyroIO {

    /*
     * +X points forward.
     * +Y points to the left.
     * +Z points to the sky.
     * Pitch is about +Y
     * Roll is about +X
     * Yaw is about +Z
     *
     * Yaw is Counterclockwise positive.
     * Nose down pitch is positive.
     * Rolling to the right is positive
     *
     * https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
     */
    private final Pigeon2 gyro;

    private final double[] rateArray = new double[3];

    public GyroIOPigeon() {
        gyro = new Pigeon2(Constants.Swerve.PIGEON_ID);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        gyro.getRawGyro(rateArray);
        inputs.yaw = gyro.getYaw();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
        inputs.yawRate = rateArray[2];
        inputs.pitchRate = rateArray[1];
        inputs.rollRate = rateArray[0];
        inputs.connected = gyro.getFirmwareVersion() > 0;
        // TODO: I would assume that 0 or below is what is returned if the device is not connected
        // but this needs to be tested.
    }

    @Override
    public void zeroGyro() {
        gyro.setYaw(0);
    }

    @Override
    public void preMatchZero(double degrees) {
        gyro.setYaw(degrees);
    }
}
