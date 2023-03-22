package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {

    private final CommandGenericHID commandDriverController;
    private final CommandXboxController commandOperatorController;
    private final Trigger unbound = new Trigger(() -> false);

    private GamePiece mode = GamePiece.CUBE;

    public enum GamePiece {
        CUBE, CONE
    }

    public Controller(int driverControllerPort, int operatorControllerPort) {
        this.commandDriverController = new CommandGenericHID(driverControllerPort);
        this.commandOperatorController = new CommandXboxController(operatorControllerPort);

        signalCone().onTrue(Commands.runOnce(() -> mode = GamePiece.CONE).ignoringDisable(true));
        signalCube().onTrue(Commands.runOnce(() -> mode = GamePiece.CUBE).ignoringDisable(true));
    }

    public Trigger coneInTrigger() {
        return commandOperatorController.leftBumper();
    }
    public Trigger coneOutTrigger() {
        return commandOperatorController.rightBumper();
    }
    public Trigger cubeInTrigger() {
        return unbound;
    }
    public Trigger cubeOutTrigger() {
        return unbound;
    }

    public Trigger signalCube() {
        return commandOperatorController.back();
    }
    public Trigger signalCone() {
        return commandOperatorController.start();
    }

    public Trigger getArmGroundTrigger() {
        return unbound;
    }

    public Trigger scoreModeTrigger() {
        return commandOperatorController.povDown();
    }

    public Trigger primeHybridTrigger() {
        return scoreModeTrigger().and(commandOperatorController.a());
    }

    public Trigger primeMidConeTrigger() {
        return scoreModeTrigger().and(() -> mode == GamePiece.CONE).and(commandOperatorController.x());
    }

    public Trigger primeHighConeTrigger() {
        return scoreModeTrigger().and(() -> mode == GamePiece.CONE).and(commandOperatorController.y());
    }

    public Trigger primeMidCubeTrigger() {
        return scoreModeTrigger().and(() -> mode == GamePiece.CUBE).and(commandOperatorController.x());
    }

    public Trigger primeHighCubeTrigger() {
        return scoreModeTrigger().and(() -> mode == GamePiece.CUBE).and(commandOperatorController.y());
    }

    public Trigger wristLevelTrigger() {
        return commandOperatorController.b();
    }

    public Trigger wristDownTrigger() {
        return commandOperatorController.a();
    }

    public double getArmSpeed() {
        return -.2
                * MathUtil.applyDeadband(Constants.OIConstants.inputCurve.apply(
                        commandOperatorController.getHID().getLeftY()), .1);
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

    public Trigger armTo20Trigger() {
        return commandOperatorController.x();
    }

    public Trigger armTo40Trigger() {
        return commandOperatorController.y();
    }
}
