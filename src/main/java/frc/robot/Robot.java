// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import java.nio.file.Path;
import java.io.IOException;

import frc.robot.lib.TankDriveConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Servo;


// Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  PWMTalonFX talonFX = new PWMTalonFX(5);
  String trajectoryJSON = "paths/testPath.wpilib.json";
  Trajectory trajectory = new Trajectory();
  Joystick joystick01 = new Joystick(1);
  Joystick joystick02 = new Joystick(0);
  Joystick joystick03 = new Joystick(2);

    private PWMTalonSRX frontLeftMotor = new PWMTalonSRX(0);
    private PWMTalonSRX backLeftMotor = new PWMTalonSRX(TankDriveConstants.LEFT_MOTOR_02);

    private PWMTalonSRX frontRightMotor = new PWMTalonSRX(TankDriveConstants.RIGHT_MOTOR_01);
    private PWMTalonSRX backRightMotor = new PWMTalonSRX(TankDriveConstants.RIGHT_MOTOR_02);    
    private PWMVictorSPX shooterAngleMotor = new PWMVictorSPX(4);

  private Servo ballRelease = new Servo(6);
     
    protected NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight");

     
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  } 


  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // SmartDashboard.putBoolean("target", shooter.isValidTarget());
    // shooter.angleShooterHood(-1);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    ballRelease.set(0.10);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive(joystick02.getY(), joystick01.getY());
    SmartDashboard.putNumber("Joy2.Y", joystick02.getY());
    SmartDashboard.putNumber("Joy1.Y", joystick01.getY());

    if (joystick03.getTriggerPressed()) {
      // Change to 1.0 once camera board is raised.
      talonFX.set(1.0);
    } 
    
    if (joystick03.getTriggerReleased()) {
      talonFX.set(0.0);
    }

    if (joystick03.getRawButtonPressed(4)) {
      shooterAngleMotor.set(0.5);
    }

    if (joystick03.getRawButtonReleased(4)) {
      shooterAngleMotor.set(0.0);
    }

    if (joystick03.getRawButtonPressed(3)) {
      shooterAngleMotor.set(-0.5);
    }

    if (joystick03.getRawButtonReleased(3)) {
      shooterAngleMotor.set(0.0);
    }

    if (joystick03.getRawButtonPressed(5)) {
      ballRelease.set(0.10);
    }

    if (joystick03.getRawButtonReleased(5)) {
      ballRelease.set(0.10);
    }

    if (joystick03.getRawButtonPressed(6)) {
      ballRelease.set(0.5);
    }

    if (joystick03.getRawButtonReleased(6)) {
      ballRelease.set(0.5);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  public void drive(double leftPower, double rightPower) {
    this.frontLeftMotor.set(leftPower);
    this.backLeftMotor.set(leftPower);

    this.frontRightMotor.set(-rightPower);
    this.backRightMotor.set(-rightPower);
  }
}
