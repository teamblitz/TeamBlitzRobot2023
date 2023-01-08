package frc.lib.oi;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// We do lambdas here to avoid un nessesary logic in the get method, Instead we let the button super
// do it.
public class ButtonBinder {

    /**
     * Returns a trigger that is active when the specified digital button is active.
     *
     * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
     * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
     */
    public static Trigger bindButton(GenericHID joystick, int buttonNumber) {
        requireNonNullParam(joystick, "joystick", "bindButton");
        return new Trigger(() -> joystick.getRawButton(buttonNumber));
    }

    /**
     * Returns a trigger that is active when the specified analog input is above 0.5 where 1 is
     * fully active.
     *
     * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
     * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
     */
    public static Trigger bindAxis(GenericHID joystick, int buttonNumber) {
        requireNonNullParam(joystick, "joystick", "bindAxis");
        return new Trigger(() -> joystick.getRawAxis(buttonNumber) >= .5);
    }

    /**
     * Returns a trigger that is active when the specified xbox button is active.
     *
     * @param joystick The XboxController.
     * @param button XboxController Button enum{@link XboxController.Button}.
     */
    public static Trigger bindButton(XboxController joystick, XboxController.Button button) {
        requireNonNullParam(joystick, "joystick", "bindButton");
        requireNonNullParam(button, "button", "bindButton");
        return new Trigger(() -> joystick.getRawButton(button.value));
    }

    /**
     * Returns a trigger that is active when the specified xbox axis is active.
     *
     * @param joystick The XboxController.
     * @param axis XboxController Axis enum{@link XboxController.Axis}.
     */
    public static Trigger bindButton(XboxController joystick, XboxController.Axis axis) {
        requireNonNullParam(joystick, "joystick", "bindButton");
        requireNonNullParam(axis, "axis", "bindButton");
        return new Trigger(() -> joystick.getRawAxis(axis.value) >= .5);
    }

    /**
     * Returns a trigger that is active when the specified PS4 button is active.
     *
     * @param joystick The PS4 Controller.
     * @param button PS4Controller Button enum{@link XboxController.Button}.
     */
    public static Trigger bindButton(PS4Controller joystick, PS4Controller.Button button) {
        requireNonNullParam(joystick, "joystick", "bindButton");
        requireNonNullParam(button, "button", "bindButton");
        return new Trigger(() -> joystick.getRawButton(button.value));
    }

    /**
     * Returns a trigger that is active when the specified xbox axis is active.
     *
     * @param joystick The PS4Controller.
     * @param button PS4Controller Axis enum{@link PS4Controller.Axis}.
     */
    public static Trigger bindButton(PS4Controller joystick, PS4Controller.Axis axis) {
        requireNonNullParam(joystick, "joystick", "bindButton");
        requireNonNullParam(axis, "axis", "bindButton");
        return new Trigger(() -> joystick.getRawAxis(axis.value) >= .5);
    }

    /**
     * Returns a trigger that is active when the specified button is active.
     *
     * @param joystick The Controller.
     * @param button XboxController Button enum{@link XboxController.Button}.
     */
    public static Trigger bindButton(SaitekX52Joystick joystick, SaitekX52Joystick.Button button) {
        requireNonNullParam(joystick, "joystick", "bindButton");
        requireNonNullParam(button, "button", "bindButton");
        return new Trigger(() -> joystick.getRawButton(button.value));
    }

    /**
     * Returns a trigger that is active when the specified xbox axis is active.
     *
     * @param joystick The XboxController.
     * @param axis XboxController Axis enum{@link XboxController.Axis}.
     */
    public static Trigger bindButton(SaitekX52Joystick joystick, SaitekX52Joystick.Axis axis) {
        requireNonNullParam(joystick, "joystick", "bindButton");
        requireNonNullParam(axis, "axis", "bindButton");
        return new Trigger(() -> joystick.getRawAxis(axis.value) >= .5);
    }
}
