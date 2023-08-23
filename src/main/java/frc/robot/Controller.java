package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.oi.SaitekX52Joystick;

/**
 * This was a very interesting experiment/way to do things. Overall, I think it worked fairly well,
 * however it has gotten messy quick. I liked moving the actual button to trigger binding out of
 * robot container, it makes things a lot cleaner, (we had about 10 buttons with maw, and that was
 * near unmanageable) we have 30, and it is a lot cleaner. However, having to dig though this class
 * to change bindings is kind of tedious I know why we decided to do it this way and not have
 * bindings in constants (cough cough analog triggers) I think it might be better to have them in
 * constants, but this worked as well, and I am not opposed to keeping this as is with some
 * organization.
 */
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

    /* Arm/Wrist/Intake manipulator controls */

    public Trigger intake() {
        return commandOperatorController
                .leftBumper()
                .or(commandDriverController.button(SaitekX52Joystick.Button.kB.value));
    }

    public Trigger slowIntakeTrigger() {
        return basicModeTrigger().and(commandOperatorController.y());
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

    /* Modes */

    public Trigger scoreModeTrigger() {
        return commandOperatorController.povDown();
    }

    public Trigger basicModeTrigger() {
        // return unbound;
        return commandOperatorController.povCenter();
    }

    public Trigger pickupModeTrigger() {
        return commandOperatorController.povUp();
    }

    public Trigger substationModeTrigger() {
        return commandOperatorController.povLeft();
    }

    /* Arm/wrist positions */

    /* Scoring presets */

    public Trigger primeHybridTrigger() {
        return pickupModeTrigger().and(commandOperatorController.b());
    }

    public Trigger primeMidConeTrigger() {
        return scoreModeTrigger().and(commandOperatorController.x());
    }

    public Trigger primeHighConeTrigger() {
        return scoreModeTrigger().and(commandOperatorController.y());
    }

    public Trigger primeMidCubeTrigger() {
        return scoreModeTrigger().and(commandOperatorController.a());
    }

    public Trigger primeHighCubeTrigger() {
        return scoreModeTrigger().and(commandOperatorController.b());
    }

    /* Substation pickups  */

    public Trigger primeConeShelfTrigger() {
        return substationModeTrigger().and(commandOperatorController.y());
    }

    public Trigger primeCubeShelfTrigger() {
        return substationModeTrigger().and(commandOperatorController.b());
    }

    public Trigger primeConeRampTrigger() {
        return substationModeTrigger().and(commandOperatorController.x());
    }

    public Trigger primeCubeRampTrigger() {
        return substationModeTrigger().and(commandOperatorController.a());
    }

    /* Ground pickup */

    public Trigger groundCubePickupTrigger() {
        return pickupModeTrigger().and(commandOperatorController.a());
    }

    public Trigger groundConeUprightPickupTrigger() {
        return pickupModeTrigger().and(commandOperatorController.y());
    }

    public Trigger groundConeFallenPickupTrigger() {
        return pickupModeTrigger().and(commandOperatorController.x());
    }

    public Trigger homeArmTrigger() {
        return basicModeTrigger().and(commandOperatorController.b());
//        return unbound;
    }

    public Trigger wristLevelTrigger() {
        return commandOperatorController.y().and(basicModeTrigger());
    }

    public Trigger wristDownTrigger() {
        return commandOperatorController.x().and(basicModeTrigger());
    }

    public Trigger retractArmTrigger() {
        return unbound;
        // return basicModeTrigger().and(commandOperatorController.a());
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
                * MathUtil.applyDeadband(
                        Constants.OIConstants.inputCurve.apply(
                                commandOperatorController.getHID().getRightY()),
                        .1);
    }

    public double getExtensionSpeed() {
        return .8
                * MathUtil.applyDeadband(
                        (Constants.OIConstants.inputCurve.apply(
                                -commandOperatorController.getHID().getLeftTriggerAxis()
                                        + commandOperatorController
                                                .getHID()
                                                .getRightTriggerAxis())),
                        0.10);
    }

    public Trigger getStartTrigger() {
        return commandOperatorController.start();
    }

    public Trigger getBackTrigger() {
        return commandOperatorController.back();
    }

    // Driver Controls
    public Trigger restGyroTrigger() {
        return commandDriverController.button(5);
    }

    public Trigger xBrakeTrigger() {
        return commandDriverController.button(3);
    }

    public Trigger brakeModeTrigger() {
        return commandDriverController.button(4);
    }

    public Trigger coastModeTrigger() {
        return commandDriverController.button(6);
    }
}
