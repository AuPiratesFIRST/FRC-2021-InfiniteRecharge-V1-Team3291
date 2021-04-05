package frc.robot.lib;

public class TankDriveConstants {
    public static final int LEFT_MOTOR_01 = 0;
    public static final int LEFT_MOTOR_02 = 1;
    public static final int RIGHT_MOTOR_01 = 2;
    public static final int RIGHT_MOTOR_02 = 3;
    
    public static final int JOYSTICK_01 = 1;
    public static final int JOYSTICK_02 = 4;
    public static final int JOYSTICK_03 = 3;    

    // Encoder ports/channels
    protected static final int[] LEFT_ENCODER_01 = {0, 1};
    public static final boolean LEFT_ENCODER_REVERESED_01 = false;

    protected static final int[] RIGHT_ENCODER_01 = {2, 3};
    public static final boolean RIGHT_ENCODER_REVERSED_01 = true;

    // PID values
    public static final double K_PROPORTIONAL = 0.00;
    public static final double K_INTEGRAL     = 0.00;
    public static final double K_DERIVATIVE   = 0.00;

    protected TankDriveConstants() {
        // Needed to remove warnings.
    }

    public static int[] getLeftEncoders() {
        return LEFT_ENCODER_01;
    }

    public static int[] getRightEncoders() {
        return RIGHT_ENCODER_01;
    }
}
