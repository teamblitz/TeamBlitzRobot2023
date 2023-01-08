package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/** Keeps track of motor status and conveys errors to the dashboard */
public class StatusManager implements Runnable {
    private static StatusManager instance;

    private final int errorCooldownMs = 2000;

    private final Map<Integer, String> canMotorStatus = new HashMap<>();
    private final Map<Integer, Long> lastError = new HashMap<>();

    private final Set<MotorController> motors = new HashSet<>();

    private final Map<MotorController, GenericEntry> motorStatus = new HashMap<>();
    private final ShuffleboardTab tab = Shuffleboard.getTab("Motors");

    private StatusManager() {}

    public static StatusManager getInstance() {
        if (instance == null) {
            instance = new StatusManager();
        }
        return instance;
    }

    /** Should Run Periodically 5 times a second */
    @Override
    public void run() {
        checkMotors();
    }

    public void logRevError(CANSparkMax motor) {
        logRevError(motor.getLastError(), motor.getDeviceId());
    }

    public void logRevError(REVLibError error, int id) {
        canMotorStatus.put(id, error.toString());
        if (error == REVLibError.kOk) return;
        if (lastError.get(id) == null
                || lastError.get(id) + errorCooldownMs < System.currentTimeMillis()) {
            System.out.println("Error: " + error.toString() + " on Spark + " + id);
            lastError.put(id, System.currentTimeMillis());
        }
    }

    public void logCTREError(BaseMotorController motor) {
        logCTREError(motor.getLastError(), motor.getDeviceID());
    }

    public void logCTREError(ErrorCode errorCode, int deviceId) {
        canMotorStatus.put(deviceId, errorCode.toString());
        if (errorCode == ErrorCode.OK) return;
        if (lastError.get(deviceId) == null
                || lastError.get(deviceId) + errorCooldownMs < System.currentTimeMillis()) {
            System.out.println("Error: " + errorCode.toString() + " on CTRE Motor + " + deviceId);
            lastError.put(deviceId, System.currentTimeMillis());
        }
    }

    public void addMotor(MotorController motor, String name) {
        motors.add(motor);
        motorStatus.put(
                motor,
                tab.add(name + " status", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry());
    }

    private void checkMotors() {
        for (MotorController motor : motors) {
            if (motor instanceof CANSparkMax) { // If the motor is a sparkMax
                CANSparkMax sparkMax = (CANSparkMax) motor;
                /* If the spark returns 0 as a firmware version,  */
                if (sparkMax.getFirmwareVersion() == 0) {
                    motorStatus.get(motor).setBoolean(false);
                } else {
                    motorStatus.get(motor).setBoolean(true);
                }
                SmartDashboard.putString(sparkMax.getDeviceId() + "", sparkMax.getFirmwareString());
                /* Eventually we could check/log motor faults here. */

            } else if (motor instanceof BaseMotorController) { // If the motor is a CTRE motor
                BaseMotorController CTREMotor = (BaseMotorController) motor;
                // TODO: This has not been tested on the talons.
                if (CTREMotor.getFirmwareVersion() == 0) {
                    motorStatus.get(motor).setBoolean(false);
                } else {
                    motorStatus.get(motor).setBoolean(true);
                }

                /* Eventually we could check/log motor faults here. */
            }
        }
    }
}
