package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.oi.SaitekX52Joystick;

public class Controller {

    private final CommandGenericHID commandDriverController;
    private final CommandXboxController commandOperatorController;
    private final Trigger unbound = new Trigger(() -> false);

    private GamePiece mode = GamePiece.CUBE;

    public enum GamePiece {
        CUBE,
        CONE
    }

    public Controller(int driverControllerPort, int operatorControllerPort) {
        this.commandDriverController = new CommandGenericHID(driverControllerPort);
        this.commandOperatorController = new CommandXboxController(operatorControllerPort);

        signalCone().onTrue(Commands.runOnce(() -> mode = GamePiece.CONE).ignoringDisable(true));
        signalCube().onTrue(Commands.runOnce(() -> mode = GamePiece.CUBE).ignoringDisable(true));
    }

    public Trigger intake() {
        return commandOperatorController
                .leftBumper()
                .or(commandDriverController.button(SaitekX52Joystick.Button.kB.value));
    }

    public Trigger outtake() {
        return commandOperatorController
                .rightBumper()
                .or(commandDriverController.button(SaitekX52Joystick.Button.kC.value));
    }

    public Trigger coneInTrigger() {
        return intake().and(cone());
    }

    public Trigger coneOutTrigger() {
        return outtake().and(cone());
    }

    public Trigger cubeInTrigger() {
        return intake().and(cube());
    }

    public Trigger cubeOutTrigger() {
        return outtake().and(cube());
    }

    public Trigger signalCube() {
        return commandOperatorController.back();
    }

    public Trigger signalCone() {
        return commandOperatorController.start();
    }

    public Trigger cone() {
        return new Trigger(() -> mode == GamePiece.CONE);
    }

    public Trigger cube() {
        return new Trigger(() -> mode == GamePiece.CUBE);
    }

    public Trigger getArmGroundTrigger() {
        return unbound;
    }

    public Trigger scoreModeTrigger() {
        return commandOperatorController.povDown();
    }

    public Trigger basicModeTrigger() {
        return commandDriverController.povCenter();
    }

    public Trigger pickupModeTrigger() {
        return commandOperatorController.povLeft();
    }

    public Trigger primeHybridTrigger() {
        return scoreModeTrigger().and(commandOperatorController.a());
    }

    public Trigger primeMidConeTrigger() {
        return scoreModeTrigger().and(cone()).and(commandOperatorController.x());
    }

    public Trigger primeHighConeTrigger() {
        return scoreModeTrigger().and(cone()).and(commandOperatorController.y());
    }

    public Trigger primeMidCubeTrigger() {
        return scoreModeTrigger().and(cube()).and(commandOperatorController.x());
    }

    public Trigger homeArmTrigger() {
        return scoreModeTrigger().and(commandOperatorController.b());
    }

    public Trigger primeHighCubeTrigger() {
        return scoreModeTrigger().and(cube()).and(commandOperatorController.y());
    }

    public Trigger primeConeShelfTrigger() {
        return pickupModeTrigger().and(cone()).and(commandOperatorController.y());
    }

    public Trigger primeCubeShelfTrigger() {
        return pickupModeTrigger().and(commandOperatorController.x());
    }

    public Trigger primeCubeSlideTrigger() {
        return pickupModeTrigger().and(commandOperatorController.b());
    }

    public Trigger groundCubePickupTrigger() {
        return pickupModeTrigger().and(commandOperatorController.a());
    }

    public Trigger wristLevelTrigger() {
        return commandOperatorController.b().and(basicModeTrigger());
    }

    public Trigger wristDownTrigger() {
        return commandOperatorController.a().and(basicModeTrigger());
    }

    public Trigger retractArmTrigger() {
        return commandOperatorController.y().and(basicModeTrigger());
    }

    public double getArmSpeed() {
        return -.2
                * MathUtil.applyDeadband(
                        Constants.OIConstants.inputCurve.apply(
                                commandOperatorController.getHID().getLeftY()),
                        .1);
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

    // Driver Controls
    public Trigger restGyroTrigger() {
        return commandDriverController.button(SaitekX52Joystick.Button.kFire.value);
    }

    public Trigger xBrakeTrigger() {
        return commandDriverController.button(SaitekX52Joystick.Button.kA.value);
    }

    public Trigger brakeModeTrigger() {
        return commandDriverController.button(SaitekX52Joystick.Button.kModeBlue.value);
    }

    public Trigger coastModeTrigger() {
        return brakeModeTrigger().negate();
    }
}
