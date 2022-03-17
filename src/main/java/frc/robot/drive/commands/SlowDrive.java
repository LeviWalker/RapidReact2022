package frc.robot.drive.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.drive.Drivetrain;

public class SlowDrive extends InstantCommand {
    public SlowDrive(Drivetrain drivetrain) {
        super(() -> drivetrain.setMaxSpeeds(
                        DriveConstants.kSlowMaxThrottle,
                        DriveConstants.kSlowMaxTurn
                    )
        );
    }
}
