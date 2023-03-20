/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
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
import frc.robot.commands.auto.AutonomousPathCommand;
import frc.robot.subsystems.Leds.LedSubsystem;
import frc.robot.subsystems.arm.ArmIOTalon;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.drive.gyro.GyroIONavx;
import frc.robot.subsystems.intake.IntakeIOSimple;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.wrist.WristIOSpark;
import frc.robot.subsystems.wrist.WristSubsystem;
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
    private WristSubsystem wristSubsystem;
    private IntakeSubsystem intakeSubsystem;

    /* ***** --- Controllers --- ***** */

    private Controller controller;
    private SaitekX52Joystick driveController;

    private final Logger logger = Logger.getInstance();

    public RobotContainer() {
        configureSubsystems();

        configureButtonBindings();
        setDefaultCommands();

        CameraServer.startAutomaticCapture();

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
                        },
                        armSubsystem));
        wristSubsystem.setDefaultCommand(
                Commands.run(
                        () -> wristSubsystem.setRotationSpeed(controller.getWristSpeed()),
                        wristSubsystem));
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
    // new SwerveModuleIOSparkMax(Constants.Swerve.Mod0.CONSTANTS),
    // new SwerveModuleIOSparkMax(Constants.Swerve.Mod1.CONSTANTS),
    // new SwerveModuleIOSparkMax(Constants.Swerve.Mod2.CONSTANTS),
    // new SwerveModuleIOSparkMax(Constants.Swerve.Mod3.CONSTANTS)
    private void configureSubsystems() {
        driveSubsystem =
                new DriveSubsystem(
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod0.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod1.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod2.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod3.CONSTANTS),
                        new GyroIONavx());

        armSubsystem = new ArmSubsystem(new ArmIOTalon());
        wristSubsystem = new WristSubsystem(new WristIOSpark(), armSubsystem);
        intakeSubsystem = new IntakeSubsystem(new IntakeIOSimple());

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

        controller.getConeInTrigger().whileTrue(intakeSubsystem.buildConeInCommand());
        controller.getConeOutTrigger().whileTrue(intakeSubsystem.buildConeOutCommand());
        controller.getCubeInTrigger().whileTrue(intakeSubsystem.buildCubeInCommand());
        controller.getCubeOutTrigger().whileTrue(intakeSubsystem.buildCubeOutCommand());
        controller.wristLevelTrigger().whileTrue(wristSubsystem.rotateRelativeToCommand(0));
        controller.wristDownTrigger().whileTrue(wristSubsystem.rotateRelativeToCommand(-90));


        controller.signalDropCone().whileTrue(LedSubsystem.buildDropConeCommand());
        controller.signalDropCube().whileTrue(LedSubsystem.buildDropCubeCommand());
        controller.signalConeSlideDrop().whileTrue(LedSubsystem.buildConeSlideDropCommand());
        controller.signalCubeSlideDrop().whileTrue(LedSubsystem.buildCubeSlideDropCommand());
        controller.signalConeLeftShelf().whileTrue(LedSubsystem.buildConeLeftShelfCommand());
        controller.signalConeRightShelf().whileTrue(LedSubsystem.buildConeRightShelfCommand());
        controller.signalCubeLeftShelf().whileTrue(LedSubsystem.buildCubeLeftShelfCommand());
        controller.signalCubeRightShelf().whileTrue(LedSubsystem.buildCubeRightShelfCommand());

        ButtonBinder.bindButton(driveController, SaitekX52Joystick.Button.kFire)
                .onTrue(Commands.runOnce(driveSubsystem::zeroGyro));
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
        AutonomousPathCommand autonomousPathCommand =
                new AutonomousPathCommand(driveSubsystem, armSubsystem, intakeSubsystem);
        return autonomousPathCommand.getFullAuto();
        // return autonomousPathCommand.balanceChargeStation();
    }
}
