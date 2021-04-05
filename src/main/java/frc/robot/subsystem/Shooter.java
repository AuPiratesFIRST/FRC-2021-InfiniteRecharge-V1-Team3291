package frc.robot.subsystem;

// Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// Shooter motor
import edu.wpi.first.wpilibj.PWMTalonFX;

// Shooter hood angle
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
// Logging
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Joystick;

public class Shooter {
    /*
    // Goal height in inches
    private static final double GOAL_HEIGHT = 96;

    // Camera height in inches
    private static final double CAMERA_HEIGHT = 18;

    // RoboRIO port
    private static final int SHOOTER_ANGLE_MOTOR_01 = 4;

    // RoboRIO port
    private static final int SHOOTER_MOTOR_01 = 5;
    
    protected double distance;

    protected NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight");

    private final PWMVictorSPX shooterAngleMotor = new PWMVictorSPX(SHOOTER_ANGLE_MOTOR_01);
    private final PWMTalonFX shooterMotor = new PWMTalonFX(SHOOTER_MOTOR_01);

    private static final double GREEN_ZONE_START = 0.0;
    private static final double GREEN_ZONE_STOP = 90.0;

    private static final double YELLOW_ZONE_START = 90.1;
    private static final double YELLOW_ZONE_STOP = 150.0;

    private static final double BLUE_ZONE_START = 150.1;
    private static final double BLUE_ZONE_STOP = 210.0;

    private static final double RED_ZONE_START = 210.1;
    private static final double RED_ZONE_STOP = 270.0;

    private int forward = 1;
    private int reverse = -1;

    Joystick joystick01;
    Joystick joystick02;

    DriveTrain mDriveTrain;

    public Shooter(Joystick joystick1, Joystick joystick2, DriveTrain mDriveTrain) {
        this.joystick01 = joystick1;
        this.joystick02 = joystick2;

        this.mDriveTrain = mDriveTrain;
    }

    public void autoShoot() {
        if (this.isValidTarget()) {
            this.distance = calculateDistance();
            */
            /*
            if (this.distance >= RED_ZONE_START && this.distance <= RED_ZONE_STOP) {
                // Figure out how much to turn the bot to be straight shot to the target

                // Turn bot to be more straight with the target
                mDriveTrain.rotateRobot(getGoalAngle());
            } else if (this.distance >= BLUE_ZONE_START && this.distance <= BLUE_ZONE_STOP) {
                // Turn bot to be more straight with the target
                mDriveTrain.rotateRobot(getGoalAngle());
            } else if (this.distance >= YELLOW_ZONE_START && this.distance <= YELLOW_ZONE_STOP) {
                // Angle shooter so it can hit the target
                angleShooterHood(this.reverse, 20);

                // Turn bot to be more straight with the target
                mDriveTrain.rotateRobot(getGoalAngle());
            } else if (this.distance >= GREEN_ZONE_START && this.distance <= GREEN_ZONE_STOP) { 
                // Angle shooter so it can hit the target
                angleShooterHood(this.forward, 20);

                // Turn bot to be more straight with the target
                mDriveTrain.rotateRobot(getGoalAngle());
            }
            */
/*
            // Shoot whether valid target or not
            shooterMotor.set(1.0);
        }
        
        shooterMotor.set(1.0);
    }    

    public void stopShooterMotor() {
        shooterMotor.set(0.0);
    }

    // Estimate distance to target
    public double calculateDistance() {
        double angleToTarget;
        double cameraAngle;
        double distanceToGoal;

        angleToTarget = getGoalAngle();
        cameraAngle = 1;

        distanceToGoal = (GOAL_HEIGHT - CAMERA_HEIGHT) / Math.tan(cameraAngle + angleToTarget);

        return distanceToGoal;
    }

    // Gets angle of the target from center of the camera
    public double getGoalAngle() {
        return camera.getEntry("tx").getDouble(0.0);
    }

    // Check if camera has a valid target
    public boolean isValidTarget() {;
        return camera.getEntry("tv").getBoolean(false);
    }

    public void angleShooterHood(int direction) {
        // Set default speed
        double speed = 0.25;

        // Direction is Forward (1) or Reverse(-1)
        speed = speed * direction;

        // Turn angle motor on
        this.shooterAngleMotor.set(speed);
    }
    */
}
