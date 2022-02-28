// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.util.sysid.CharacterizeDrive;
import frc.robot.util.vision.VisionClient;
import frc.robot.util.vision.VisionClient.VisionClientException;
import frc.robot.util.control.*;
import frc.robot.util.oi.NKPS4Controller;
import frc.robot.util.tunable.NKSmartNumber;
import frc.robot.util.tunable.Tunable;
import frc.robot.util.tunable.NKSmartBoolean;


public class Robot extends TimedRobot {

  NKPS4Controller driver, operator;

  Compressor compressor;

  ColorMatch match;
  
  NKTalonFX m7;

  double targetRPM = 0;

  final double maxThrottle = 0.80, maxTurn = 0.70;
  double flywheelPower = 0.40;
  double intakePower = 0.00; // 0.50
  double transferPower = 0.20;
  double testDrivePower = 0.30;
  double climbPower = 0.50;
  boolean out = false;

  NKVictorSPX m10;

  Servo lServo, rServo, testServo;

  NKSmartNumber motorPower;

  VisionClient vision;

  String ip = "pi@wpilibpi";
  short port = 5050;

  ColorSensorV3 color;

  NKDoubleSolenoid intake;
  NKSolenoid gear;

  // private final double kClimbP = 0.00256640574875;

  // private final double climbExtended = 247.793, climbRetracted = 0;

  private static boolean red, blue;

  @Override
  public void robotInit() {

    red = DriverStation.getAlliance() == Alliance.Red;
    blue = !red;

    // SmartDashboard.putBoolean("isRedAlliance", red);

    // CameraServer.startAutomaticCapture("Camera 0", 0);
    // CameraServer.startAutomaticCapture("Camera 1", 1);

    // SmartDashboard.putNumber("rpm", targetRPM);

    // try {
    //   vision = new VisionClient(ip, port);
    //   vision.start();
    // } catch (VisionClientException e) {
    //   e.printStackTrace();
    // }

    color = new ColorSensorV3(Port.kOnboard);
    // color.configureColorSensor(
    //   ColorSensorResolution.kColorSensorRes20bit,
    //   ColorSensorMeasurementRate.kColorRate25ms,
    //   GainFactor.kGain1x
    // );

    driver = new NKPS4Controller(0);
    operator = new NKPS4Controller(1);

    compressor = new Compressor(41, PneumaticsModuleType.REVPH);
    compressor.enableDigital();

    intake = new NKDoubleSolenoid(41, PneumaticsModuleType.REVPH, 2, 3);

    // gear = new NKSolenoid(41, PneumaticsModuleType.REVPH, 4);

    // lServo = new Servo(0);
    // rServo = new Servo(1);

    m10 = new NKVictorSPX(10); // transfer

    // SmartDashboard.putBoolean("intake", intake.get() == Value.kForward);
    SmartDashboard.putBoolean("high gear", gear.get());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    double smartDashIntakeSpeed = SmartDashboard.getNumber("intakeSpeed", intakePower);

    if (intakePower != smartDashIntakeSpeed) {
      intakePower = smartDashIntakeSpeed;
      SmartDashboard.putNumber("intakeSpeed", intakePower);
      System.out.println(">>>>>>> Changed intake speed to " + intakePower + " <<<<<<<");
    }

    String detectedColor = null;

    if (color.getRed() > 10000) {
      detectedColor = "Red";
    } else if (color.getBlue() > 10000) {
      detectedColor = "Blue";
    } else {
      detectedColor = "Unknown";
    }

    SmartDashboard.putString("color sensor color", detectedColor);
    double[] rgb = {color.getRed(), color.getGreen(), color.getBlue()};
    SmartDashboard.putNumberArray("Color RGB values", rgb);
  }

  @Override
  public void autonomousInit() {
    // new CharacterizeDrive(new DrivetrainSubsystem()).schedule();

    // new DriveDistanceCommand(drive, 1).schedule();

    // new DriveStraightCommand(drive).schedule();

  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  public static boolean isRedAlliance() {
    return red;
  }
  public static boolean isBlueAlliance() {
    return blue;
  }
}
