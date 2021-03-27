package frc.robot.subsystem;

// Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// Shooter motor
import edu.wpi.first.wpilibj.PWMTalonFX;

// Shooter hood angle
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;

// Logging
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    // Goal height in inches
    private static final double GOAL_HEIGHT = 96;

    // Camera height in inches
    private static final double CAMERA_HEIGHT = 18;

    // Camera angle in degrees
    private static final double CAMERA_ANGLE = 20;

    // RoboRIO port
    private static final int SHOOTER_ANGLE_MOTOR_01 = 6;

    // RoboRIO port
    private static final int SHOOTER_MOTOR_01 = 7;
    
    protected double distance;

    protected NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight");

    private final PWMVictorSPX shooterAngleMotor = new PWMVictorSPX(SHOOTER_ANGLE_MOTOR_01);
    private final PWMTalonFX shooterMotor = new PWMTalonFX(SHOOTER_MOTOR_01);
    
    // Creates an ADXRS450_Gyro object on the MXP SPI port
    Gyro gyro = new ADXRS450_Gyro(SPI.Port.kMXP);

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

    public void autoShoot() {
        if (this.isValidTarget()) {

            this.distance = calculateDistance();
 
            if (this.distance >= RED_ZONE_START && this.distance <= RED_ZONE_STOP) {
                // Not sure if we need this yet.
            } else if (this.distance >= BLUE_ZONE_START && this.distance <= BLUE_ZONE_STOP) {
                // Not sure if we need this yet.
            } else if (this.distance >= YELLOW_ZONE_START && this.distance <= YELLOW_ZONE_STOP) {
                angleShooterHood(this.reverse, 20);
            } else if (this.distance >= GREEN_ZONE_START && this.distance <= GREEN_ZONE_STOP) {  
                angleShooterHood(this.forward, 20);
            }
            
            shooterMotor.set(1.0);
        }
    }    

    public double calculateDistance() {
        double angleToTarget;
        double distanceToGoal;

        angleToTarget = getGoalAngle();

        distanceToGoal = (GOAL_HEIGHT - CAMERA_HEIGHT) / Math.tan(CAMERA_ANGLE + angleToTarget);

        return distanceToGoal;
    }

    public double getGoalAngle() {
        return camera.getEntry("tx").getDouble(0.0);
    }

    public boolean isValidTarget() {
        return camera.getEntry("tv").getBoolean(false);
    }

    public void angleShooterHood(int direction, double angle) {
        // Set default speed
        double speed = 0.25;
        double gyroAngle;

        // Direction is Forward (1) or Reverse(-1)
        speed = speed * direction;

        // Turn angle motor on
        this.shooterAngleMotor.set(speed);

        do {
            // Check angle constantly to determine if we're where we want to be
            gyroAngle = this.gyro.getAngle();

            // Tell me what the angle is
            SmartDashboard.putNumber("Shooter Angle", gyroAngle);
            SmartDashboard.putNumber("Shooter Speed", speed);
        } while (gyroAngle <= angle);

        // Stop motor once we hit the desired angle
        this.shooterAngleMotor.set(0.0);
    }
}
