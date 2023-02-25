package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
    private final XboxController xboxController;
    // Constructor
    public Controller(XboxController xboxController) {
        this.xboxController = xboxController;
    }

    public Trigger getArmGroundTrigger() {

        return null;
    }

    public Trigger getArmLowConeTrigger() {
        return null;
    }
    public Trigger getArmMidConeTrigger() {
        return null;
    }
    public Trigger getArmHighConeTrigger() {
        return null;
    }

    public Trigger getArmLowCubeTrigger() {
        return null;
    }
    public Trigger getArmMidCubeTrigger() {
        return null;
    }
    public Trigger getArmHighCubeTrigger() {
        return null;
    }
}
