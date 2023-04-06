package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoBalance extends CommandBase {

    DriveSubsystem driveSubsystem;

    Timer minTimer = new Timer();

    Timer waitTimer = new Timer();

    boolean hasDecremented;
    double speed;

    public AutoBalance(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        minTimer.start();
        waitTimer.start();
        hasDecremented = false;
        speed = .45;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double xOut = 0;
        if (Math.abs(driveSubsystem.getPitch())
                > Constants.AutoConstants.CHARGE_STATION_MAX_ANGLE) {
            xOut = Math.signum(-driveSubsystem.getPitch()) * speed;
        } else if (minTimer.get() < 1) {
            xOut = Math.signum(-driveSubsystem.getPitch()) * speed;
        }

        driveSubsystem.drive(new Translation2d(xOut, 0), 0, false, false, true);

        if (Math.abs(driveSubsystem.getPitch())
                < Constants.AutoConstants.CHARGE_STATION_MIN_ANGLE) {
            if (!hasDecremented) {
                speed += -.05;
                hasDecremented = true;
            }
        } else {
            hasDecremented = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(new Translation2d(0, 0), 0, false, false, false);
        System.out.println("Auto Balance over");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!(Math.abs(driveSubsystem.getPitch())
                < Constants.AutoConstants.CHARGE_STATION_MIN_ANGLE)) {
            waitTimer.reset();
        }
        return Math.abs(driveSubsystem.getPitch())
                        < Constants.AutoConstants.CHARGE_STATION_MIN_ANGLE
                && (minTimer.get() > 1)
                && (waitTimer.get() > .4);
    }
}
