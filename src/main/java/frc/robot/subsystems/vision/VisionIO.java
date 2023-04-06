package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {

    public class VisionIOInputs implements LoggableInputs {
        public double[][] poses;
        public double[][] rotMats;
        public double[] timestamps;

        @Override
        public void toLog(LogTable table) {
            table.put("timestamps", timestamps);
            table.put("count", poses.length);
            for (int i = 0; i < poses.length; i++) {
                table.put("poses/" + i, poses[i]);
            }
            for (int i = 0; i < rotMats.length; i++) {
                table.put("rotMats/" + i, rotMats[i]);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            timestamps = table.getDoubleArray("timestamps", timestamps);
            int frameCount = (int) table.getInteger("count", 0);
            poses = new double[frameCount][];
            rotMats = new double[frameCount][];
            for (int i = 0; i < frameCount; i++) {
                poses[i] = table.getDoubleArray("pose/" + i, new double[] {});
            }
            for (int i = 0; i < frameCount; i++) {
                rotMats[i] = table.getDoubleArray("rotMats/" + i, new double[] {});
            }
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIO.VisionIOInputs inputs) {}
}
