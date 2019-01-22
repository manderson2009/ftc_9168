package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ma61444 on 2/2/2018.
 */
public abstract class BaseAutoMode extends LinearOpMode {

    RoverRuckusHardware robot   = new RoverRuckusHardware();   // Use a Pushbot's hardware

    /* Declare OpMode members. */
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.6;
    static final double     DRIVE_SPEED_IN_PER_SEC  = 61.5/3; // Corresponds to setting of 0.6 above
    static final double     DRIVE_ACCEL_CORRECTION  = 3.5;//inches

    static final double     TURN_SPEED              = 0.5;
    static final double     TURN_SPEED_DEG_PER_SEC  = 180/2.5;// Corresponds to setting of 0.5 above

    public boolean redTeam = true;

    public void InitLoopWaitForStartButton()
    {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.flipper.setPosition(0);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    }



    public void driveForward(double inches, boolean doAccel)
    {
        ElapsedTime timer = new ElapsedTime();
        double milliseconds = inches / DRIVE_SPEED_IN_PER_SEC * 1000;
        double driveSpeed = 0;

        timer.reset();
        if(doAccel)
        {
            while (driveSpeed < DRIVE_SPEED) {
                robot.setLeftDrive(driveSpeed);
                robot.setRightDrive(driveSpeed);

                driveSpeed += .1;
                robot.setLeftDrive(driveSpeed);
                robot.setRightDrive(driveSpeed);
                sleep(100);
            }

            /* There is some distance lost due to accelerating. Account for that here */
            milliseconds += DRIVE_ACCEL_CORRECTION / DRIVE_SPEED_IN_PER_SEC * 1000;
        }
        else
        {
            driveSpeed = DRIVE_SPEED;
        }

        robot.setLeftDrive(driveSpeed);
        robot.setRightDrive(driveSpeed);

        /* Sleep rest of duration */
        if(milliseconds > timer.milliseconds())
        {
            sleep((long)(milliseconds - timer.milliseconds()));
        }

        robot.motorsOff();
    }

    public void turnLeft(double degrees)
    {
        double milliseconds = degrees / TURN_SPEED_DEG_PER_SEC * 1000;
        robot.rotateLeft(TURN_SPEED);

        sleep((long)milliseconds);
        robot.motorsOff();
    }

    public void turnRight(double degrees)
    {
        double milliseconds = degrees / TURN_SPEED_DEG_PER_SEC * 1000;
        robot.rotateRight(TURN_SPEED);

        sleep((long)milliseconds);
        robot.motorsOff();
    }


}
