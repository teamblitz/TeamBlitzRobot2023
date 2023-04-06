package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.EnumSet;

public class VisionIONTJetson implements VisionIO {
    public VisionIONTJetson() {
        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
        DoubleSubscriber poseSub = ntInst.getDoubleTopic("jetson/pose").subscribe(0);

//        ntInst.addListener(poseSub, EnumSet.of(NetworkTableEvent.Kind.kPublish))
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {

    }
}
