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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ManipulatorCommandFactory;
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

    private ManipulatorCommandFactory manipulatorCommandFactory;

    /* ***** --- Controllers --- ***** */

    private Controller controller;
    //     private SaitekX52Joystick driveController;
    private Joystick driveController;

    // private final Logger logger = Logger.getInstance();

    /* ***** --- Autonomous --- ***** */
    // *** Must match with path names in pathplanner folder ***
    private static final String[] autonomousCommands = {
        "Left1sM",
        "Left2sB",
        "Left2sHM",
        "Left2sMB",
        "MiddleL1sB",
        "MiddleL1sMB",
        "MiddleR1sB",
        "MiddleR1sMB",
        "MiddleC1sMB",
        "Right1sM",
        "Right2sHM",
        "Right2sMB",
        "Score",
        "SquareTest",
        "BalanceTest",
        "Nothing"
    };
    private final SendableChooser<String> chooser = new SendableChooser<>();

    public RobotContainer() {
        configureSubsystems();

        manipulatorCommandFactory =
                new ManipulatorCommandFactory(armSubsystem, intakeSubsystem, wristSubsystem);

        configureButtonBindings();
        setDefaultCommands();

//        CameraServer.startAutomaticCapture();

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
                                        -driveController.getY() * calculateDriveMultiplier()),
                        () ->
                                OIConstants.inputCurve.apply(
                                        -driveController.getX() * calculateDriveMultiplier()),
                        () -> OIConstants.inputCurve.apply(-driveController.getTwist()) * .3,
                        () -> false,
                        () -> driveController.getPOV()));

        armSubsystem.rotationRequirement.setDefaultCommand(
                new HoldArmAtPositionCommand(armSubsystem));
        wristSubsystem.setDefaultCommand(
                Commands.waitSeconds(0).andThen(wristSubsystem.holdAtRelativeCommand()));
    }

    private final SlewRateLimiter driveMultiplierLimiter = new SlewRateLimiter(.25);

    private double calculateDriveMultiplier() {
        if (driveController.getRawButton(2)) {
            return driveMultiplierLimiter.calculate(.3);
        } else if (driveController.getRawButton(1)) {
            return driveMultiplierLimiter.calculate(1);
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

        armSubsystem =
                new ArmSubsystem(
                        new ArmIOTalon(),
                        () -> wristSubsystem.getRelativeRotation(),
                        () -> wristSubsystem.getRotation());
        wristSubsystem =
                new WristSubsystem(
                        new WristIOSpark(),
                        () -> armSubsystem.getRotation(),
                        () -> armSubsystem.getArmLength());
        intakeSubsystem = new IntakeSubsystem(new IntakeIOSimple());
        ledSubsystem = new LedSubsystem();

        driveController = new Joystick(0); // Move this to Controller (and I never did)
        controller = new Controller(0, 1);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        controller
                .coneInTrigger()
                .whileTrue(intakeSubsystem.buildConeInCommand())
                .onFalse(intakeSubsystem.slowIntakeCommand());
        controller.coneOutTrigger().whileTrue(intakeSubsystem.buildConeOutCommand());
        controller
                .cubeInTrigger()
                .whileTrue(intakeSubsystem.buildCubeInCommand())
                .onFalse(intakeSubsystem.slowIntakeCommand());
        controller.cubeOutTrigger().whileTrue(intakeSubsystem.buildCubeOutCommand());
        controller.slowIntakeTrigger().whileTrue(intakeSubsystem.slowIntakeCommand());

        controller.wristLevelTrigger().onTrue(wristSubsystem.rotateRobotRelativeToCommand(0));
        controller.wristDownTrigger().onTrue(wristSubsystem.rotateRobotRelativeToCommand(-90));

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
                                                }))
                                .finallyDo((b) -> wristSubsystem.setRotationSpeed(0)));

        new Trigger(() -> Math.abs(controller.getArmSpeed()) > .02)
                .whileTrue(
                        Commands.run(
                                () -> {
                                    armSubsystem.setArmRotationSpeed(controller.getArmSpeed());
                                },
                                armSubsystem.rotationRequirement));

        new Trigger(() -> Math.abs(controller.getExtensionSpeed()) > .05)
                .whileTrue(
                        Commands.run(
                                        () -> {
                                            armSubsystem.setArmExtensionSpeed(
                                                    controller.getExtensionSpeed());
                                        },
                                        armSubsystem.extensionRequirement)
                                .finallyDo((b) -> armSubsystem.setArmExtensionSpeed(0)));

        controller.primeHybridTrigger().onTrue(manipulatorCommandFactory.primeHybrid());

        controller.primeMidConeTrigger().onTrue(manipulatorCommandFactory.primeConeMid());
        controller.primeHighConeTrigger().onTrue(manipulatorCommandFactory.primeConeHigh());

        controller.primeMidCubeTrigger().onTrue(manipulatorCommandFactory.primeCubeMid());
        controller.primeHighCubeTrigger().onTrue(manipulatorCommandFactory.primeCubeHigh());

        controller.homeArmTrigger().onTrue(armSubsystem.homeArmCommand());
        controller.retractArmTrigger().onTrue(armSubsystem.retractArmCommand());

        controller.primeConeShelfTrigger().onTrue(manipulatorCommandFactory.primeConeShelf());
        controller.primeCubeShelfTrigger().onTrue(manipulatorCommandFactory.primeCubeShelf());

        controller.primeCubeRampTrigger().onTrue(manipulatorCommandFactory.primeCubeRamp());
        controller.primeConeRampTrigger().onTrue(manipulatorCommandFactory.primeConeRamp());

        controller.groundCubePickupTrigger().onTrue(manipulatorCommandFactory.groundCubePickup());
        controller
                .groundConeUprightPickupTrigger()
                .onTrue(manipulatorCommandFactory.groundUprightConePickup());
        controller
                .groundConeFallenPickupTrigger()
                .onTrue(manipulatorCommandFactory.groundFallenConePickup());

        controller.cone().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("mode", true)));
        controller.cube().onTrue(Commands.runOnce(() -> SmartDashboard.putBoolean("mode", false)));

        controller
                .brakeModeTrigger()
                .onTrue(Commands.runOnce(() -> driveSubsystem.setBrakeMode(true)));
        controller
                .coastModeTrigger()
                .onTrue(Commands.runOnce(() -> driveSubsystem.setBrakeMode(false)));

        controller.getStartTrigger().whileTrue(driveSubsystem.driveSpeedTestCommand(1, 4));
        controller.getBackTrigger().whileTrue(driveSubsystem.driveSpeedTestCommand(-1, 4));
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
        Command wristFix = Commands.runOnce(wristSubsystem::setHoldGoals);
        wristFix.schedule();

        String autoCommand = chooser.getSelected();
        AutonomousPathCommand autonomousPathCommand =
                new AutonomousPathCommand(
                        driveSubsystem, armSubsystem, intakeSubsystem, manipulatorCommandFactory);
        return autonomousPathCommand.generateAutonomous(autoCommand);
    }
}
