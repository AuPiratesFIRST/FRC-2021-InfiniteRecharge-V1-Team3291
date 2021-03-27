package frc.robot.subsystem;

// Motor controllers
import edu.wpi.first.wpilibj.PWMTalonSRX;

// Drive train classes
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.lib.TankDriveConstants;
import edu.wpi.first.wpilibj.Encoder;

// NavX2 Gryoscope/Accelerameter/Magnetometter
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;

public class DriveTrain {
    private final SpeedController frontLeftMotor = new PWMTalonSRX(TankDriveConstants.LEFT_MOTOR_01);
    private final SpeedController backLeftMotor = new PWMTalonSRX(TankDriveConstants.LEFT_MOTOR_02);
    private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);

    private final SpeedController frontRightMotor = new PWMTalonSRX(TankDriveConstants.RIGHT_MOTOR_01);
    private final SpeedController backRightMotor = new PWMTalonSRX(TankDriveConstants.RIGHT_MOTOR_02);
    private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);
    
    private DifferentialDrive tankDriveTrain = new DifferentialDrive(leftMotors, rightMotors);

    int[] leftEncoderChannels = TankDriveConstants.getLeftEncoders();
    private Encoder leftEncoder = new Encoder(
        leftEncoderChannels[0],
        leftEncoderChannels[1],
        TankDriveConstants.LEFT_ENCODER_REVERESED_01
    );

    int[] rightEncoderChannels = TankDriveConstants.getRightEncoders();
    private Encoder rightEncoder = new Encoder(
        rightEncoderChannels[0],
        rightEncoderChannels[1],
        TankDriveConstants.RIGHT_ENCODER_REVERSED_01
    );

    double heading;
    double kP = 1;

    private static final double WHEEL_DIAMETER = 6;
    private static final double ROBOT_DIAMETER = 46;

    // One foot per encoder rotation
    private static double distancePerPulse = 1.0 / 256.0;

    double movementPerDegree = ((ROBOT_DIAMETER / WHEEL_DIAMETER) * distancePerPulse) / 360;

    public DriveTrain() {
        this.resetEncoders();

        this.leftEncoder.setDistancePerPulse(distancePerPulse);
        this.rightEncoder.setDistancePerPulse(distancePerPulse);

        this.zeroHeading();

        this.heading = this.gyroAhrs.getAngle();
    }

    /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
    /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
    /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
    AHRS gyroAhrs = new AHRS(Port.kUSB1); 

    public void drive(double x, double y) {
        double error = this.heading - this.gyroAhrs.getAngle();

        tankDriveTrain.tankDrive(x + (this.kP * error), y + (this.kP * error));
    }

    public void driveToPosition() {

    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    public double getAverageEncoderDistance() {
        return ((leftEncoder.getDistance() + rightEncoder.getDistance()) / 2);
    }

    public void zeroHeading() {
        gyroAhrs.reset();
    }

    public double getHeading() {
        return this.heading;
    }

    public void setHeading() {
        this.heading = this.gyroAhrs.getAngle();
    }

    public double[] rotateRobot(double angle) {
        double[] position = {0.0, 0.0};



        return position;
    }
}
