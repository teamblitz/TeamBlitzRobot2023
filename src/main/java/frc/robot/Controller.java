package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
    private final CommandXboxController commandOperatorController;
    private final Trigger unbound = new Trigger(() -> false);

    public Controller(int driverControllerPort, int operatorControllerPort) {
        this.commandOperatorController = new CommandXboxController(operatorControllerPort);
    }

    public Trigger getConeInTrigger() {
        return commandOperatorController.leftBumper();
    }

    public Trigger getConeOutTrigger() {
        return commandOperatorController.rightBumper();
    }

    public Trigger getCubeInTrigger() {
        return unbound;
    }

    public Trigger getCubeOutTrigger() {
        return unbound;
    }

    public Trigger getArmGroundTrigger() {
        return unbound;
    }

    public Trigger getArmLowConeTrigger() {
        return unbound;
    }

    public Trigger getArmMidConeTrigger() {
        return unbound;
    }

    public Trigger getArmHighConeTrigger() {
        return unbound;
    }

    public Trigger getArmLowCubeTrigger() {
        return unbound;
    }

    public Trigger getArmMidCubeTrigger() {
        return unbound;
    }

    public Trigger getArmHighCubeTrigger() {
        return unbound;
    }

    public double getArmSpeed() {
        return -.2
                * Constants.OIConstants.inputCurve.apply(
                        commandOperatorController.getHID().getLeftY());
    }

    public double getWristSpeed() {
        return -.2
                * Constants.OIConstants.inputCurve.apply(
                        commandOperatorController.getHID().getRightX());
    }

    public double getExtensionSpeed() {
        return .3
                * (Constants.OIConstants.inputCurve.apply(
                        commandOperatorController.getHID().getLeftTriggerAxis()
                                - commandOperatorController
                                        .getHID()
                                        .getRightTriggerAxis())); // Differential arm control
    }
}
