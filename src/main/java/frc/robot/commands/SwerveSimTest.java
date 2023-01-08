package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SwerveSimTest extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final GenericEntry translationEntry;
    private final GenericEntry strafeEntry;
    private final GenericEntry rotationEntry;

    public SwerveSimTest(DriveSubsystem s_Swerve) {
        this.driveSubsystem = s_Swerve;
        addRequirements(s_Swerve);

        ShuffleboardTab tab = Shuffleboard.getTab("DriveSubsystem");
        translationEntry = tab.add("Translation", 0).getEntry();
        strafeEntry = tab.add("Strafe", 0).getEntry();
        rotationEntry = tab.add("Rotation", 0).getEntry();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveSubsystem.drive(
                new Translation2d(translationEntry.getDouble(0), strafeEntry.getDouble(0)),
                rotationEntry.getDouble(0),
                false,
                true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
