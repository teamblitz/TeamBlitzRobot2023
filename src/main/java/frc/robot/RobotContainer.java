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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.oi.SaitekX52Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CommandBuilder;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.arm.HoldArmAtPositionCommand;
import frc.robot.commands.auto.AutonomousPathCommand;
import frc.robot.subsystems.arm.ArmIOTalon;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.drive.gyro.GyroIONavx;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;
import frc.robot.subsystems.intake.IntakeIOSimple;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LedSubsystem;
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
    private LedSubsystem ledSubsystem;

    private CommandBuilder commandBuilder;

    /* ***** --- Controllers --- ***** */

    private Controller controller;
    private SaitekX52Joystick driveController;

    private final Logger logger = Logger.getInstance();

    /* ***** --- Autonomous --- ***** */
    private static final String[] autonomousCommands = {
        "Left", "Middle", "Right", "Test", "Nothing"
    };
    private final SendableChooser<String> chooser = new SendableChooser<>();

    public RobotContainer() {
        configureSubsystems();

        commandBuilder =
                new CommandBuilder(
                        armSubsystem,
                        driveSubsystem,
                        intakeSubsystem,
                        ledSubsystem,
                        wristSubsystem);

        configureButtonBindings();
        setDefaultCommands();

        CameraServer.startAutomaticCapture();

        DriverStation.silenceJoystickConnectionWarning(true);
        Shuffleboard.getTab("DriveSubsystem")
                .add(
                        "ResetOdometry",
                        Commands.runOnce(() -> driveSubsystem.resetOdometry(new Pose2d())));

        for (String auto : autonomousCommands) {
            chooser.addOption(auto, auto);
        }
        chooser.setDefaultOption("Nothing", "Nothing");
        SmartDashboard.putData("Autonomous Choices", chooser);
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
        armSubsystem.RotationRequirement.setDefaultCommand(
                new HoldArmAtPositionCommand(armSubsystem));
        // wristSubsystem.setDefaultCommand(
        // Commands.waitSeconds(.8).andThen(wristSubsystem.holdAtCommand()));
        //        wristSubsystem.setDefaultCommand(
        //        Commands.run(
        //                () -> wristSubsystem.setRotationSpeed(controller.getWristSpeed()),
        //                wristSubsystem)
        //        );
        wristSubsystem.setDefaultCommand(
                Commands.waitSeconds(0).andThen(wristSubsystem.holdAtRelativeCommand()));
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
                        Constants.Swerve.USE_PIGEON ? new GyroIOPigeon() : new GyroIONavx());

        armSubsystem = new ArmSubsystem(new ArmIOTalon());
        wristSubsystem = new WristSubsystem(new WristIOSpark(), armSubsystem);
        intakeSubsystem = new IntakeSubsystem(new IntakeIOSimple());
        ledSubsystem = new LedSubsystem();

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

        controller.coneInTrigger().whileTrue(intakeSubsystem.buildConeInCommand());
        controller.coneOutTrigger().whileTrue(intakeSubsystem.buildConeOutCommand());
        controller.cubeInTrigger().whileTrue(intakeSubsystem.buildCubeInCommand());
        controller.cubeOutTrigger().whileTrue(intakeSubsystem.buildCubeOutCommand());
        controller.wristLevelTrigger().onTrue(wristSubsystem.rotateRelativeToCommand(0));
        controller.wristDownTrigger().onTrue(wristSubsystem.rotateRelativeToCommand(-90));

        controller.signalCube().whileTrue(ledSubsystem.cubeSolid());
        controller.signalCone().whileTrue(ledSubsystem.coneSolid());

        controller.restGyroTrigger().onTrue(Commands.runOnce(driveSubsystem::zeroGyro));
        controller.xBrakeTrigger().onTrue(driveSubsystem.buildParkCommand());

        new Trigger(() -> Math.abs(controller.getWristSpeed()) > .02)
                .whileTrue(
                        Commands.run(
                                        () ->
                                                wristSubsystem.setRotationSpeed(
                                                        controller.getWristSpeed()),
                                        wristSubsystem)
                                .alongWith(
                                        Commands.run(
                                                () -> {
                                                    wristSubsystem.lastRelativeGoal =
                                                            wristSubsystem.getRelativeRotation();
                                                    wristSubsystem.lastGoal =
                                                            wristSubsystem.getRotation();
                                                })));

        new Trigger(() -> Math.abs(controller.getArmSpeed()) > .02)
                .whileTrue(
                        Commands.run(
                        () -> {
                            armSubsystem.setArmRotationSpeed(controller.getArmSpeed());
                        },
                        armSubsystem.RotationRequirement)
                );
        
        new Trigger(() -> Math.abs(controller.getExtensionSpeed()) > .02)
                .whileTrue(
                        Commands.run(
                        () -> {
                            armSubsystem.setArmExtensionSpeed(controller.getExtensionSpeed());
                        },
                        armSubsystem.ExtensionRequirement).finallyDo((b) -> armSubsystem.setArmExtensionSpeed(0))
                );

        // controller.armTo20Trigger().whileTrue(armSubsystem.extendToCommand(.2));
        // controller.armTo40Trigger().whileTrue(armSubsystem.extendToCommand(.5));
        
        controller.primeHybridTrigger().onTrue(commandBuilder.primeHybrid());
        controller.primeMidConeTrigger().onTrue(commandBuilder.primeConeMid());
        controller.primeHighConeTrigger().onTrue(commandBuilder.primeConeMid());

        controller.primeMidCubeTrigger().onTrue(commandBuilder.primeCubeMid());
        controller.primeHighCubeTrigger().onTrue(commandBuilder.primeCubeMid());

        controller.homeArmTrigger().onTrue(armSubsystem.homeArmCommand());
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
        String autoCommand = chooser.getSelected();
        // System.out.println(autoCommand);
        AutonomousPathCommand autonomousPathCommand =
                new AutonomousPathCommand(
                        driveSubsystem, armSubsystem, intakeSubsystem, commandBuilder);
        switch (autoCommand) {
            case "Left":
                return autonomousPathCommand.generateAutonomous("Left");
            case "Middle":
                return autonomousPathCommand.generateAutonomous("Middle");
            case "Right":
                return autonomousPathCommand.generateAutonomous("Right");
            case "Nothing":
                return null;
            case "Test":
                return autonomousPathCommand.generateAutonomous("Test");
            default:
                return null;
        }
    }
}
