package frc.robot.drive.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.drive.Drivetrain;

public class RegularDrive extends InstantCommand {
    public RegularDrive(Drivetrain drivetrain) {
        super(() -> drivetrain.setMaxSpeeds(
                        DriveConstants.kRegularMaxThrottle,
                        DriveConstants.kRegularMaxTurn
                    )
        );
    }
}
