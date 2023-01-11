/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.oi.SaitekX52Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveTuning;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.drive.gyro.GyroIONavx;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* ***** --- Subsystems --- ***** */

    private DriveSubsystem driveSubsystem;

    /* ***** --- Controllers --- ***** */
    private XboxController driveController1;
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
        // Set default command for drive
        // driveSubsystem.setDefaultCommand(
        //         new TeleopSwerve(
        //                 driveSubsystem,
        //                 () -> -driveController.getLeftY() * .2,
        //                 () -> -driveController.getLeftX() * .2,
        //                 () -> -driveController.getRightX() * .2,
        //                 () -> false));
        driveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        driveSubsystem,
                        () ->
                                OIConstants.inputCurve.apply(-driveController.getRawAxis(SaitekX52Joystick.Axis.kYAxis.value)
                                        * calculateDriveMutiplyer()),
                        () ->
                                OIConstants.inputCurve.apply(-driveController.getRawAxis(SaitekX52Joystick.Axis.kXAxis.value)
                                        * calculateDriveMutiplyer()),
                        () -> -driveController.getRawAxis(SaitekX52Joystick.Axis.kZRot.value) * .2,
                        () -> false));
        // driveSubsystem.setDefaultCommand(tuningCommand);
        // driveSubsystem.setDefaultCommand(Commands.run(() -> {
        //     // System.out.println("Touhiuhl");
        //     logger.recordOutput("DriveController/DriveY", driveController.getRawAxis(SaitekX52Joystick.Axis.kYAxis.value));
        //     logger.recordOutput("DriveController/DriveX", driveController.getRawAxis(SaitekX52Joystick.Axis.kXAxis.value));
            
        // }, driveSubsystem
        // ));
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
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod0.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod1.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod2.CONSTANTS),
                        new SwerveModuleIOSparkMax(Constants.Swerve.Mod3.CONSTANTS),
                        new GyroIONavx());
        driveController1 = new XboxController(1);
        driveController = new SaitekX52Joystick(0);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new Trigger(() -> driveController.getRawButton(SaitekX52Joystick.Button.kFire.value))
                .onTrue(Commands.runOnce(driveSubsystem::zeroGyro));
        new Trigger(driveController1::getAButton)
                .onTrue(new InstantCommand(tuningCommand::nextAngle));
    }

    public Command getAutonomousCommand() { // Autonomous code goes here
        return null;
    }
}
