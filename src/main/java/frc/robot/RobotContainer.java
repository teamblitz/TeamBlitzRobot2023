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
import frc.lib.oi.SaitekX52Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOTalonSpark;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.drive.gyro.GyroIONavx;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSimple;
import frc.robot.subsystems.intake.IntakeSubsystem;
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

    private IntakeSubsystem intakeSubsystem;

    /* ***** --- Controllers --- ***** */

    private Controller controller;
    private SaitekX52Joystick driveController;

    private final Logger logger = Logger.getInstance();

    public RobotContainer() {
        configureSubsystems();

        configureButtonBindings();
        setDefaultCommands();

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
                                                * calculateDriveMultiplier()),
                        () ->
                                OIConstants.inputCurve.apply(
                                        -driveController.getRawAxis(
                                                        SaitekX52Joystick.Axis.kXAxis.value)
                                                * calculateDriveMultiplier()),
                        () -> -driveController.getRawAxis(SaitekX52Joystick.Axis.kZRot.value) * .2,
                        () -> false));
        armSubsystem.setDefaultCommand(
                Commands.run(
                        () -> {
                            armSubsystem.setArmRotationSpeed(controller.getArmSpeed());
                            armSubsystem.setArmExtensionSpeed(controller.getExtensionSpeed());
                            armSubsystem.setWristRotationSpeed(controller.getWristSpeed());
                        }, armSubsystem));
    }

    private final SlewRateLimiter driveMultiplierLimiter = new SlewRateLimiter(.25);

    private double calculateDriveMultiplier() {
        if (driveController.getRawButton(SaitekX52Joystick.Button.kLowerTrigger.value)) {
            return driveMultiplierLimiter.calculate(.3);
        } else if (driveController.getRawButton(SaitekX52Joystick.Button.kUpperTrigger2.value)) {
            return driveMultiplierLimiter.calculate(1);
        } else if (driveController.getRawButton(SaitekX52Joystick.Button.kUpperTrigger1.value)) {
            return driveMultiplierLimiter.calculate(.80);
        } else {
            return driveMultiplierLimiter.calculate(.60);
        }
    }

    private void configureSubsystems() {
        driveSubsystem =
                new DriveSubsystem(
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod0.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod1.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod2.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod3.CONSTANTS),
                        new GyroIONavx());

        armSubsystem = new ArmSubsystem(new ArmIO() {});
        intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});

        driveController = new SaitekX52Joystick(0); // Move this to Controller
        controller = new Controller(0, 1);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        controller.getConeInTrigger().onTrue(intakeSubsystem.buildConeInCommand());
        controller.getConeOutTrigger().onTrue(intakeSubsystem.buildConeOutCommand());
        controller.getCubeInTrigger().onTrue(intakeSubsystem.buildCubeInCommand());
        controller.getCubeOutTrigger().onTrue(intakeSubsystem.buildCubeOutCommand());

        ButtonBinder.bindButton(driveController, SaitekX52Joystick.Button.kFire).onTrue(Commands.runOnce(driveSubsystem::zeroGyro));

        // Below is mostly deprecated as we are using analog control for arm right now.

        //        ButtonBinder.bindButton(buttonBox,
        // Constants.OIConstants.ButtonBoxMappings.UP_ARM.value)
        //                .onTrue(Commands.runOnce(() ->
        // armSubsystem.setArmRotationSpeed(.2))).onFalse(Commands.runOnce(() ->
        // armSubsystem.setArmRotationSpeed(0)));
        //
        //        ButtonBinder.bindButton(buttonBox, OIConstants.ButtonBoxMappings.DOWN_ARM.value)
        //                .onTrue(Commands.runOnce(() ->
        // armSubsystem.setArmRotationSpeed(-.2))).onFalse(Commands.runOnce(() ->
        // armSubsystem.setArmRotationSpeed(0)));
        //
        //        ButtonBinder.bindButton(buttonBox, OIConstants.ButtonBoxMappings.ARM_IN.value)
        //                .onTrue(Commands.runOnce(() ->
        // armSubsystem.setArmExtensionSpeed(.3))).onFalse(Commands.runOnce(() ->
        // armSubsystem.setArmExtensionSpeed(0)));
        //
        //        ButtonBinder.bindButton(buttonBox, OIConstants.ButtonBoxMappings.ARM_OUT.value)
        //                .onTrue(Commands.runOnce(() ->
        // armSubsystem.setArmExtensionSpeed(-.3))).onFalse(Commands.runOnce(() ->
        // armSubsystem.setArmExtensionSpeed(0)));
        
        controller.getLeftExtendTrigger()
        .onTrue(Commands.runOnce(() ->
        armSubsystem.setLeftExtensionSpeed(.1))).onFalse(Commands.runOnce(() ->
        armSubsystem.setArmExtensionSpeed(0)));
        
        controller.getLeftExtendTrigger()
                       .onTrue(Commands.runOnce(() ->
        armSubsystem.setRightExtensionSpeed(.1))).onFalse(Commands.runOnce(() ->
        armSubsystem.setArmExtensionSpeed(0)));
        //
        //
        //        ButtonBinder.bindButton(buttonBox, OIConstants.ButtonBoxMappings.INTAKE_IN.value)
        //                .onTrue(new PrintCommand("IN").andThen(Commands.runOnce(() ->
        // intakeSubsystem.inCone()))).onFalse(Commands.runOnce(() -> intakeSubsystem.stop()));
        //
        //        ButtonBinder.bindButton(buttonBox, OIConstants.ButtonBoxMappings.INTAKE_OUT.value)
        //                .onTrue(new PrintCommand("IN").andThen(Commands.runOnce(() ->
        // intakeSubsystem.outCone()))).onFalse(Commands.runOnce(() -> intakeSubsystem.stop()));
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
        return null; // add commands into this later, use AutonomousPathCommand
    }
}
