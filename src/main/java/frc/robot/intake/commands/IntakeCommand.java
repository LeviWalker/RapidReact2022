package frc.robot.intake.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.indexer.Indexer;
import frc.robot.intake.Intake;

public class IntakeCommand extends CommandBase {

    private Intake intake;
    private Indexer indexer;
    private Joystick operator;

    private double intakeAxis, intakeSpeed;

    public IntakeCommand(Intake intake, Indexer indexer, Joystick operator) {
        this.intake = intake;
        this.indexer = indexer;
        this.operator = operator;
        addRequirements(intake);
        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
    }

    @Override
    public void execute() {
        intakeAxis = -operator.getRawAxis(OIConstants.kRightYJoystick);
        if (Math.abs(intakeAxis) > 0.30 && !intake.isDeployed())
            intake.deploy();
        else if (Math.abs(intakeAxis) < 0.30 && intake.isDeployed())
            intake.retract();

        intakeSpeed = SmartDashboard.getNumber("Intake Speed", intakeSpeed);

        if (Math.abs(intakeAxis) > 0.07) intake.setIntake(Math.signum(intakeAxis) * intakeSpeed);
        else intake.setIntake(0);

        // if operator wants to intake, clamp input to min and max if we have the intake deployed
        // else just set to some slow speed
        if (intakeAxis > 0.07)
            intake.setIntake((intakeAxis > 0.30)? MathUtil.clamp(intakeAxis, 0.65, 0.80) : 0.25);
        // if operator wants to outtake, clamp input to min and max if we have the intake deployed
        // else just set to some slow speed
        else if (intakeAxis < -0.07)
            intake.setIntake((intakeAxis < -0.30)? MathUtil.clamp(intakeAxis, -0.80, -0.65) : -0.25);
        // if IntakeIndexCommand is not running then ShootIndexCommand is running,
        // thus we run the intake wheels to make sure cargo are entering the indexer
        else if (!indexer.getDefaultCommand().isScheduled())
            intake.setIntake(-0.25);
        else
            intake.setIntake(0);
    }
}