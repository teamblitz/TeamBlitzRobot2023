/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        System.out.println("Robot Start up at: " + Timer.getFPGATimestamp());

        Logger logger = Logger.getInstance();

        // Record metadata
        logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source

        // Running on a real robot, try to log to a USB stick, else log to the roboRIO
        if (isReal()) {
            String logDir = Filesystem.getOperatingDirectory().getAbsolutePath();
            boolean thumbDriveConnected;
            try {
                // The /u directory maps to the connected usb drive if one exists.
                // If /u doesn't work replace it with /media/sda1
                Path usbDir = Paths.get("/media/sda1").toRealPath();
                thumbDriveConnected = Files.isWritable(usbDir);
                if (thumbDriveConnected) {
                    logDir = usbDir.toString();
                    System.out.println("USB drive connected, will log to USB drive.");
                } else {
                    System.out.printf(
                            "USB drive not found, will log to the %s directory on the roboRIO.%n",
                            logDir);
                }
            } catch (IOException e) {
                System.out.printf(
                        "An IOException occurred when checking for a USB drive, will try to log to %s directory on the roboRIO.%n",
                        logDir);
            }

            logger.addDataReceiver(new WPILOGWriter(logDir));
            logger.addDataReceiver(new NT4Publisher());

        } else
            switch (Constants.simMode) {
                    // Running a physics simulator, log to local folder
                case SIM:
                    logger.addDataReceiver(new WPILOGWriter(""));
                    logger.addDataReceiver(new NT4Publisher());
                    break;
                    // Replaying a log, set up replay source
                case REPLAY:
                    setUseTiming(false); // Run as fast as possible
                    String logPath = LogFileUtil.findReplayLog();
                    logger.setReplaySource(new WPILOGReader(logPath));
                    logger.addDataReceiver(
                            new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                    break;
            }

        // Start AdvantageKit logger
        logger.start();

        robotContainer = new RobotContainer();

        // This should be uncommented at some point.
        // try {
        //     Networker networker = new Networker();
        //     networker.start();
        //     System.out.println("Networker Started Successfully");
        // } catch (IOException e) {
        //     System.out.println(e);
        // }
    }

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /* ***** --- Autonomous --- ***** */

    // Called at the start of autonomous.
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule autonomous commands
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    // Called periodically during autonomous
    @Override
    public void autonomousPeriodic() {}

    // Called at the end of autonomous
    @Override
    public void autonomousExit() {
        // Cancel autonomous commands
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /* ***** --- Teleop --- ***** */

    // Called at the start of teleop
    @Override
    public void teleopInit() {
        System.out.println("TeleopInit");
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    // Called periodically during teleop
    @Override
    public void teleopPeriodic() {}

    // Called at the end of teleop.
    @Override
    public void teleopExit() {}

    /* ***** --- Test Mode --- ***** */

    // Called at the start of test mode
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    // Called periodically during test mode
    @Override
    public void testPeriodic() {}

    // Called at the end of test mode
    @Override
    public void testExit() {}

    /* ***** --- Disabled --- ***** */

    // Called when disabled
    @Override
    public void disabledInit() {}

    // Called periodically when disabled
    @Override
    public void disabledPeriodic() {}

    // Called when the robot exits disabled mode
    @Override
    public void disabledExit() {}

    /* ***** --- Simulation --- ***** */

    // Called when the robot enters simulation
    @Override
    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Called periodicly durring simulation
    @Override
    public void simulationPeriodic() {}
}
