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

    public Trigger getLeftExtendTrigger() {
        return commandOperatorController.pov(0);
    }

    public Trigger getRightExtendTrigger() {
        return commandOperatorController.pov(180);
    }

    public Trigger wristLevelTrigger() {
        return commandOperatorController.b();
    }

    public Trigger wristDownTrigger() {
        return commandOperatorController.a();
    }

    public double getArmSpeed() {
        return -.2
                * Constants.OIConstants.inputCurve.apply(
                        commandOperatorController.getHID().getLeftY());
    }

    public double getWristSpeed() {
        return -.2
                * Constants.OIConstants.inputCurve.apply(
                        commandOperatorController.getHID().getRightY());
    }

    public double getExtensionSpeed() {
        return .5
                * (Constants.OIConstants.inputCurve.apply(
                        -commandOperatorController.getHID().getLeftTriggerAxis()
                                + commandOperatorController
                                        .getHID()
                                        .getRightTriggerAxis())); // Differential arm control
    }

    public Trigger getStartTrigger() {
        return commandOperatorController.start();
    }

    public Trigger signalDropCone() {
        return unbound;
    }

    public Trigger signalDropCube() {
        return unbound;
    }

    public Trigger signalConeSlideDrop() {
        return unbound;
    }

    public Trigger signalCubeSlideDrop() {
        return unbound;
    }

    public Trigger signalConeLeftShelf() {
        return unbound;
    }

    public Trigger signalConeRightShelf() {
        return unbound;
    }

    public Trigger signalCubeLeftShelf() {
        return unbound;
    }

    public Trigger signalCubeRightShelf() {
        return unbound;
    }

    
}
