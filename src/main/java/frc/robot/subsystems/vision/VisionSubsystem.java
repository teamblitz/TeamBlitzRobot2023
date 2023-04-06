package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.lib.TimestampedValue;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.LinkedList;
import java.util.Queue;

public class VisionSubsystem extends SubsystemBase implements BlitzSubsystem {

    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private final Logger logger = Logger.getInstance();

    private final Deque<TimestampedValue<Pose2d>> poseQueue;


    public VisionSubsystem(VisionIO io) {
        this.io = io;
        // initial capacity 16, I don't think we will fill it up
        this.poseQueue = new ArrayDeque<>();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        logger.processInputs("vision", inputs);

        // check the inputs here
    }

    public Deque<TimestampedValue<Pose2d>> getQueue() {
        return poseQueue;
    }
}
