/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.oi.ButtonBinder;
import frc.lib.oi.ButtonBox;
import frc.lib.oi.SaitekX52Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveTuning;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.arm.ArmIOTalonSpark;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.gyro.GyroIONavx;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* ***** --- Subsystems --- ***** */

    private DriveSubsystem driveSubsystem;

    private ArmSubsystem armSubsystem;

    /* ***** --- Controllers --- ***** */
    private ButtonBox buttonBox;
    private SaitekX52Joystick driveController;

    private SwerveTuning tuningCommand;
    private final Logger logger = Logger.getInstance();

    public RobotContainer() {
        configureSubsystems();

        // CameraServer.startAutomaticCapture(); // Ignore warning.

        this.tuningCommand = new SwerveTuning(driveSubsystem);

        configureButtonBindings();
        setDefaultCommands();
        // tuningTab.add("Tuning Command", tuningCommand);

        // tuningCommand.schedule();
        DriverStation.silenceJoystickConnectionWarning(true);
        Shuffleboard.getTab("DriveSubsystem")
                .add(
                        "ResetOdometry",
                        Commands.runOnce(() -> driveSubsystem.resetOdometry(new Pose2d())));
    }

    private void setDefaultCommands() {
        driveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        driveSubsystem,
                        () ->
                                OIConstants.inputCurve.apply(
                                        -driveController.getRawAxis(
                                                        SaitekX52Joystick.Axis.kYAxis.value)
                                                * calculateDriveMutiplyer()),
                        () ->
                                OIConstants.inputCurve.apply(
                                        -driveController.getRawAxis(
                                                        SaitekX52Joystick.Axis.kXAxis.value)
                                                * calculateDriveMutiplyer()),
                        () -> -driveController.getRawAxis(SaitekX52Joystick.Axis.kZRot.value) * .2,
                        () -> false));
    }

    private final SlewRateLimiter driveMultiplyerLimiter = new SlewRateLimiter(.25);

    private double calculateDriveMutiplyer() {
        if (driveController.getRawButton(SaitekX52Joystick.Button.kLowerTrigger.value)) {
            return driveMultiplyerLimiter.calculate(.2);
        } else if (driveController.getRawButton(SaitekX52Joystick.Button.kUpperTrigger2.value)) {
            return driveMultiplyerLimiter.calculate(1);
        } else if (driveController.getRawButton(SaitekX52Joystick.Button.kUpperTrigger1.value)) {
            return driveMultiplyerLimiter.calculate(.75);
        } else {
            return driveMultiplyerLimiter.calculate(.5);
        }
    }

    private void configureSubsystems() {
        driveSubsystem =
                new DriveSubsystem(
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {},
                        new GyroIONavx());

        armSubsystem = new ArmSubsystem(new ArmIOTalonSpark());
        buttonBox = new ButtonBox(1);
        driveController = new SaitekX52Joystick(0);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        ButtonBinder.bindButton(buttonBox, Constants.OIConstants.ButtonBoxMappings.UP_ARM.value)
                .onTrue(Commands.runOnce(() -> armSubsystem.setArmRotationSpeed(.2))).onFalse(Commands.runOnce(() -> armSubsystem.setArmRotationSpeed(0)));

        ButtonBinder.bindButton(buttonBox, OIConstants.ButtonBoxMappings.DOWN_ARM.value)
                .onTrue(Commands.runOnce(() -> armSubsystem.setArmRotationSpeed(-.2))).onFalse(Commands.runOnce(() -> armSubsystem.setArmRotationSpeed(0)));
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
        return null;
    }
}
