package frc.robot.drive.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drivetrain;

public class DrivetrainCharacterization extends CommandBase {
    private Drivetrain drivetrain;

    private final double kAppliedVoltage = 7;

    private Timer timer;
    private ArrayList<Double> time, leftVelocity, rightVelocity;

    public DrivetrainCharacterization(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        timer = new Timer();
        time = new ArrayList<Double>();
        leftVelocity = new ArrayList<Double>();
        rightVelocity = new ArrayList<Double>();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        this.drivetrain.tankDriveVolts(kAppliedVoltage, kAppliedVoltage);
        // collect data on time and velocity to solve differential equation for feedforward gains
        time.add(timer.get());
        leftVelocity.add(drivetrain.getLeftEncoderVelocityMetersPerSecond());
        rightVelocity.add(drivetrain.getRightEncoderVelocityMetersPerSecond());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        drivetrain.autoPercentArcadeDrive(0, 0);

        // dump results onto SmartDashboard
        // SmartDashboard.putNumber("kAppliedVoltage", kAppliedVoltage);

        double[] timeArray = new double[time.size()];
        double[] leftVelocityArray = new double[leftVelocity.size()];
        double[] rightVelocityArray = new double[rightVelocity.size()];

        for (int i = 0; i < timeArray.length; i++) {
            timeArray[i] = time.get(i);
        }

        for (int i = 0; i < leftVelocityArray.length; i++) {
            leftVelocityArray[i] = leftVelocity.get(i);
        }

        for (int i = 0; i < rightVelocityArray.length; i++) {
            rightVelocityArray[i] = rightVelocity.get(i);
        }

        for (int i = 0; i < timeArray.length; i++) {
            System.out.println(timeArray[i] + ", " + leftVelocityArray[i] + ", " + rightVelocityArray[i]);
        }

        // SmartDashboard.putNumberArray("time", timeArray);
        // SmartDashboard.putNumberArray("leftVelocity", leftVelocityArray);
        // SmartDashboard.putNumberArray("rightVelocity", rightVelocityArray);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(15) || super.isFinished();
    }
}
