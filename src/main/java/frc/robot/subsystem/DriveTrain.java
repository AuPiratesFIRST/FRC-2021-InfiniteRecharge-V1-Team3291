package frc.robot.subsystem;

// Motor controllers
import edu.wpi.first.wpilibj.PWMTalonSRX;

// Drive train classes
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.lib.TankDriveConstants;
import edu.wpi.first.wpilibj.Encoder;

// Drive station
import edu.wpi.first.wpilibj.Joystick;

// NavX2 Gryoscope/Accelerameter/Magnetometter
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain {
    private final SpeedController frontLeftMotor = new PWMTalonSRX(TankDriveConstants.LEFT_MOTOR_01);
    private final SpeedController backLeftMotor = new PWMTalonSRX(TankDriveConstants.LEFT_MOTOR_02);
    private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);

    private final SpeedController frontRightMotor = new PWMTalonSRX(2);
    private final SpeedController backRightMotor = new PWMTalonSRX(3);
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

    Joystick joystick01 = new Joystick(TankDriveConstants.JOYSTICK_01);
    Joystick joystick02 = new Joystick(TankDriveConstants.JOYSTICK_02);

    /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
    /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
    /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
    AHRS gyroAhrs = new AHRS(SPI.Port.kMXP); 

    public void drive() {
        tankDriveTrain.tankDrive(joystick01.getX(), joystick02.getX());
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
}
