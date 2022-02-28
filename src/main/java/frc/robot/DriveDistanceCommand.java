package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistanceCommand extends CommandBase {

    DrivetrainSubsystem drive;

    PIDController leftDistanceController, rightDistanceController, headingController;

    double distanceError, headingError;

    private double kDistP = 0.5,
                   kDistI = 0.00,
                   kDistD = 0.00;
    
    private double kHeadingP = 0.01,
                   kHeadingI = 0.00,
                   kHeadingD = 0.00;

    double target;

    boolean runMotors;

    public DriveDistanceCommand(DrivetrainSubsystem drive, double meters) {

        this.target = meters;

        this.drive = drive;
        this.drive.resetEncoders();
        this.drive.getIMU().reset();

        SmartDashboard.putBoolean("runMotors", runMotors);

        kDistP = SmartDashboard.getNumber("dp", kDistP);
        kDistI = SmartDashboard.getNumber("di", kDistI);
        kDistD = SmartDashboard.getNumber("dd", kDistD);
        kHeadingP = SmartDashboard.getNumber("hp", kHeadingP);
        kHeadingI = SmartDashboard.getNumber("hi", kHeadingI);
        kHeadingD = SmartDashboard.getNumber("hd", kHeadingD);

        SmartDashboard.putNumber("dp", kDistP);
        SmartDashboard.putNumber("di", kDistI);
        SmartDashboard.putNumber("dd", kDistD);
        SmartDashboard.putNumber("hp", kHeadingP);
        SmartDashboard.putNumber("hi", kHeadingI);
        SmartDashboard.putNumber("hd", kHeadingD);

        leftDistanceController = new PIDController(kDistP, kDistI, kDistD);
        leftDistanceController.setSetpoint(meters);
        leftDistanceController.setTolerance(0.05);
        rightDistanceController = new PIDController(kDistP, kDistI, kDistD);
        rightDistanceController.setSetpoint(meters);
        rightDistanceController.setTolerance(0.05);
        headingController = new PIDController(kHeadingP, kHeadingI, kHeadingD);
        headingController.enableContinuousInput(-180, 180);
        headingController.setSetpoint(0);
    }

    @Override
    public void initialize() {
        this.drive.resetEncoders();
        this.drive.getIMU().reset();

        SmartDashboard.putBoolean("runMotors", runMotors);

        kDistP = SmartDashboard.getNumber("dp", kDistP);
        kDistI = SmartDashboard.getNumber("di", kDistI);
        kDistD = SmartDashboard.getNumber("dd", kDistD);
        kHeadingP = SmartDashboard.getNumber("hp", kHeadingP);
        kHeadingI = SmartDashboard.getNumber("hi", kHeadingI);
        kHeadingD = SmartDashboard.getNumber("hd", kHeadingD);

        SmartDashboard.putNumber("dp", kDistP);
        SmartDashboard.putNumber("di", kDistI);
        SmartDashboard.putNumber("dd", kDistD);
        SmartDashboard.putNumber("hp", kHeadingP);
        SmartDashboard.putNumber("hi", kHeadingI);
        SmartDashboard.putNumber("hd", kHeadingD);

        leftDistanceController = new PIDController(kDistP, kDistI, kDistD);
        leftDistanceController.setSetpoint(target);
        leftDistanceController.setTolerance(0.05);
        rightDistanceController = new PIDController(kDistP, kDistI, kDistD);
        rightDistanceController.setSetpoint(target);
        rightDistanceController.setTolerance(0.05);
        headingController = new PIDController(kHeadingP, kHeadingI, kHeadingD);
        headingController.enableContinuousInput(-180, 180);
        headingController.setSetpoint(0);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("left pos error", leftDistanceController.getPositionError());
        SmartDashboard.putNumber("right pos error", rightDistanceController.getPositionError());

        double dp = SmartDashboard.getNumber("dp", kDistP);
        double di = SmartDashboard.getNumber("di", kDistI);
        double dd = SmartDashboard.getNumber("dd", kDistD);
        double hp = SmartDashboard.getNumber("hp", kHeadingP);
        double hi = SmartDashboard.getNumber("hi", kHeadingI);
        double hd = SmartDashboard.getNumber("hd", kHeadingD);

        boolean run = SmartDashboard.getBoolean("runMotors", runMotors);

        if (runMotors != run) {
            runMotors = run;
            SmartDashboard.putBoolean("runMotors", runMotors);
        }

        if (kDistP != dp) {
            kDistP = dp;
            leftDistanceController.setP(dp);
            rightDistanceController.setP(dp);
        }

        if (kDistI != di) {
            kDistI = di;
            leftDistanceController.setI(di);
            rightDistanceController.setI(di);
        }

        if (kDistD != dd) {
            kDistD = dd;
            leftDistanceController.setD(dd);
            rightDistanceController.setD(dd);
        }

        if (kHeadingP != hp) {
            kHeadingP = hp;
            headingController.setP(hp);
        }

        if (kHeadingI != hi) {
            kHeadingI = hi;
            headingController.setI(hi);
        }

        if (kHeadingD != hd) {
            kHeadingD = hd;
            headingController.setD(hd);
        }

        double correction = headingController.calculate(drive.getIMU().getAngle());

        double left = leftDistanceController.calculate(drive.getLeftEncoderDistanceMeters());
        double right = rightDistanceController.calculate(drive.getRightEncoderDistanceMeters());

        SmartDashboard.putNumber("left power", left);
        SmartDashboard.putNumber("right power", right);
        SmartDashboard.putNumber("correction power", correction);

        left -= correction;
        right += correction;

        if (runMotors) drive.tankDrive(left, right);

        SmartDashboard.putNumber("dp", kDistP);
        SmartDashboard.putNumber("di", kDistI);
        SmartDashboard.putNumber("dd", kDistD);
        SmartDashboard.putNumber("hp", kHeadingP);
        SmartDashboard.putNumber("hi", kHeadingI);
        SmartDashboard.putNumber("hd", kHeadingD);
        SmartDashboard.putBoolean("runMotors", runMotors);
    }

    @Override
    public boolean isFinished() {
        return leftDistanceController.atSetpoint() && rightDistanceController.atSetpoint();
    }
}
