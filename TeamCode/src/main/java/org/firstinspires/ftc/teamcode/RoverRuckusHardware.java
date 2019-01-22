package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RoverRuckusHardware
{
    /* Public OpMode members. */
    DcMotor  leftFront   = null;
    DcMotor  leftRear  = null;
    DcMotor  rightFront    = null;
    DcMotor  rightRear    = null;

    DcMotor  lifter    = null;

    //DcMotor lifter = null;
    ColorSensor colorSensor = null;

    Servo flipper = null;

    GyroSensor gyro = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    RoverRuckusHardware(){

    }

    /* Initialize standard Hardware interfaces */
    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront   = hwMap.dcMotor.get("left_front");
        leftRear  = hwMap.dcMotor.get("left_rear");
        rightFront    = hwMap.dcMotor.get("right_front");
        rightRear    = hwMap.dcMotor.get("right_rear");

        lifter = hwMap.dcMotor.get("lifter");

        flipper = hwMap.servo.get("flipper");


        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hwMap.colorSensor.get("color_sensor");

        gyro = hwMap.gyroSensor.get("gyro");
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    void setLeftDrive(double power)
    {
        leftFront.setPower(power);
        leftRear.setPower(power);
    }

    void setRightDrive(double power)
    {
        rightFront.setPower(power);
        rightRear.setPower(power);
    }

    void driveForward(double power)
    {
        leftFront.setPower(power);
        leftRear.setPower(power);

        rightFront.setPower(power);
        rightRear.setPower(power);
    }


    void crabLeft(double power)
    {
        leftFront.setPower(-power);
        leftRear.setPower(power);

        rightFront.setPower(power);
        rightRear.setPower(-power);

    }

    void crabRight(double power)
    {
        leftFront.setPower(power);
        leftRear.setPower(-power);

        rightFront.setPower(-power);
        rightRear.setPower(power);
    }

    void rotateLeft(double power)
    {
        leftFront.setPower(-power);
        leftRear.setPower(-power);

        rightFront.setPower(power);
        rightRear.setPower(power);

    }

    void rotateRight(double power)
    {
        leftFront.setPower(power);
        leftRear.setPower(power);

        rightFront.setPower(-power);
        rightRear.setPower(-power);
    }

    public void motorsOff()
    {
        setLeftDrive(0);
        setRightDrive(0);
    }

}

