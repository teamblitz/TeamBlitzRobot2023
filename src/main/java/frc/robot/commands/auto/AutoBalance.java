package frc.robot.commands.auto;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoBalance extends CommandBase {

    DriveSubsystem driveSubsystem;

    Timer minTimer = new Timer();

    public AutoBalance(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        minTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double xOut = 0;
        if (Math.abs(driveSubsystem.getPitch()) > Constants.AutoConstants.CHARGE_STATION_MAX_ANGLE) {
            xOut = Math.signum(-driveSubsystem.getPitch()) * 0.6;
        } else if (minTimer.get() < 1) {
            xOut = Math.signum(-driveSubsystem.getPitch()) * 0.6;
        }


        driveSubsystem.drive(new Translation2d(
                xOut, 0), 0, false, false, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
//        driveSubsystem.drive();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(driveSubsystem.getPitch()) < Constants.AutoConstants.CHARGE_STATION_MIN_ANGLE && (minTimer.get() > 1);
    }
}