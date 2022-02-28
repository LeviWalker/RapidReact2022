package frc.robot.util.sysid;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.Drivetrain;

public class CharacterizeDrive extends CommandBase {
    private final Drivetrain m_drivebase;
    private SysIdDrivetrainLogger m_logger;   
    private Double m_prevAngle = 0.0;
    private Double m_prevTime = 0.0;
    private boolean m_resetComplete;
    public CharacterizeDrive(Drivetrain subsystem) {

        m_drivebase = subsystem;
        addRequirements(m_drivebase);   
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // reset gyro and encoders
        // set timeperiod to .005
        m_drivebase.getDifferentialDrive().setDeadband(0.0);
        // The following is called for the side-effect of resetting the 
        // drivebase odometers.
        m_drivebase.resetOdometry(m_drivebase.getOdometry().getPoseMeters()); 
        m_logger = new SysIdDrivetrainLogger();
        m_logger.updateThreadPriority();
        m_logger.initLogging();
        m_resetComplete = false;
    }
   
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftPosition = m_drivebase.getLeftEncoderDistanceMeters();
        double leftRate = m_drivebase.getLeftEncoderVelocityMetersPerSecond();
        double rightPosition = m_drivebase.getRightEncoderDistanceMeters();
        double rightRate = m_drivebase.getRightEncoderVelocityMetersPerSecond();
        double angularPosition = -Math.toRadians(m_drivebase.getIMU().getAngle());
        double deltaAngle = angularPosition - m_prevAngle;
        double now = Timer.getFPGATimestamp();
        double deltaTime = now - m_prevTime;
        double angularRate = m_prevTime==0 || deltaTime==0 ? 0.0 : deltaAngle/deltaTime;
        m_prevAngle = angularPosition;
        m_prevTime = now;

        // Resetting encoders takes non-zero time on CAN-based encoders
        // Wait for the reset to complete
        if (!m_resetComplete) {
            if (leftPosition > 0.01 || rightPosition > 0.01) return;
            m_resetComplete = true;
        }
        m_logger.log(leftPosition, rightPosition, leftRate, 
                   rightRate, angularPosition, angularRate);

        if (!m_logger.willRotate()) {

            m_drivebase.driveStraightVolts(m_logger.getLeftMotorVoltage());

        } else {
            m_drivebase.tankDriveVolts(
                m_logger.getLeftMotorVoltage(),
                m_logger.getRightMotorVoltage()
            ); 
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Characterization done; disabled");
        m_drivebase.tankDrive(0,0);
        m_logger.sendData();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
