package frc.robot.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.networktables.NetworkTableEntry;
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

    private final SpeedController shooterAngleMotor = new PWMVictorSPX(SHOOTER_ANGLE_MOTOR_01);
    private final PWMTalonFX shooterMotor = new PWMTalonFX(SHOOTER_MOTOR_01);

    private double greenZoneStart = 0.0;
    private double greenZoneStop = 90.0;

    private double yellowZoneStart = 90.1;
    private double yellowZoneStop = 150.0;

    private double blueZoneStart = 150.1;
    private double blueZoneStop = 210.0;

    private double redZoneStart = 210.1;
    private double redZoneStop = 270.0;

    public void autoShoot() {
        if (this.isValidTarget()) {

            this.distance = calculateDistance();
 
            if (this.distance >= redZoneStart && this.distance <= redZoneStop) {

            } else if (this.distance >= blueZoneStart && this.distance <= blueZoneStop) {
            
            } else if (this.distance >= yellowZoneStart && this.distance <= yellowZoneStop) {
            
            } else if (this.distance >= greenZoneStart && this.distance <= greenZoneStop) {                

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
}
