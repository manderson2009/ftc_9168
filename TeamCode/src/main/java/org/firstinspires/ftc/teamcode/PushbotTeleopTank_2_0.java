/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import static java.lang.Math.abs;


/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@TeleOp(name="Pushbot: Teleop Tank 2.0", group="Thunderbot")
public class PushbotTeleopTank_2_0 extends OpMode{

    /* Declare OpMode members. */
    private HardwarePushbot2_0 robot       = new HardwarePushbot2_0(); // use the class created to define a Pushbot's hardware


    private boolean crawlMode = false;
    private boolean aWasPressed = false;

    private boolean resettingLifter = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

        // Set all motors to zero power
        robot.leftFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightRear.setPower(0);

        robot.gyro.calibrate();
        while(robot.gyro.isCalibrating())
        {
            //wait for calibration to finish
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.colorSensor.enableLed(true);
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.openGrabberServo();
        robot.stowBallServo();
    }


    private void MoveLifter(boolean up)
    {
        if (up && robot.lifter.getCurrentPosition() < 5920) {
            LifterUp();
        } else if(!up && robot.lifter.getCurrentPosition() > 0) {
            LifterDown();
        }
        else
        {
            LifterOff();
        }
    }

    private void LifterUp()
    {
        robot.lifter.setPower(1);
    }

    private void LifterDown()
    {
        robot.lifter.setPower(-1);
    }

    private void LifterOff()
    {
        robot.lifter.setPower(0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //Right trigger close servo
        if (gamepad2.right_trigger > 0) {
            robot.closeGrabberServo();
        }

        //Left trigger open servo
        if (gamepad2.left_trigger > 0) {
            robot.openGrabberServo();
        }

        if(gamepad2.left_bumper && gamepad2.back)
        {
            robot.lifter.setPower(-.5);
            resettingLifter = true;
        }
        else if(resettingLifter)
        {
            resettingLifter = false;
            robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else if (gamepad2.left_bumper) {
            MoveLifter(false);
        }else if (gamepad2.right_bumper) {
            MoveLifter(true);
        }
        else {
            LifterOff();
        }
        telemetry.addData("motor encoder ",robot.lifter.getCurrentPosition());


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        if (gamepad1.a && !aWasPressed)
            crawlMode = !crawlMode;
        aWasPressed = gamepad1.a;


        double r = Math.sqrt(Math.pow(gamepad1.left_stick_x,2) + Math.pow(gamepad1.left_stick_y,2));
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        double left_front = r * Math.cos(robotAngle) + rightX;
        double right_front = r * Math.sin(robotAngle) - rightX;
        double left_rear = r * Math.sin(robotAngle) + rightX;
        double right_rear = r * Math.cos(robotAngle) - rightX;

        double maximum = Math.max(abs(left_front), abs(right_front));
        maximum = Math.max(maximum, abs(left_rear));
        maximum = Math.max(maximum, abs(right_rear));

        double magnitude = Math.max(1,maximum);

        double crawl_speed = 1.0;
        if(crawlMode)
        {
            crawl_speed = 2;
        }
        robot.leftFront.setPower(left_front/magnitude/crawl_speed);
        robot.rightFront.setPower(right_front/magnitude/crawl_speed);
        robot.leftRear.setPower(left_rear/magnitude/crawl_speed);
        robot.rightRear.setPower(right_rear/magnitude/crawl_speed);


        telemetry.addData("gyro reading: ",robot.gyro.getHeading());
        telemetry.addData("Color sensor red: ", robot.colorSensor.red());
        telemetry.addData("Color sensor blue: ", robot.colorSensor.blue());
        telemetry.addData("Color sensor green: ", robot.colorSensor.green());

/*
        bCurrState = gamepad1.x;

        // check for button state transitions.
        if ((bCurrState == true) && (bCurrState != bPrevState))  {

            // button is transitioning to a pressed state. So Toggle LED
            bLedOn = !bLedOn;
            robot.colorSensor.enableLed(bLedOn);
        }

        // update previous state variable.
        bPrevState = bCurrState;
        Color.RGBToHSV(robot.colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);
        telemetry.addData("Clear", robot.colorSensor.alpha());
        telemetry.addData("Red  ", robot.colorSensor.red());
        telemetry.addData("Green", robot.colorSensor.green());
        telemetry.addData("Blue ", robot.colorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addLine(" ");
        if(robot.touchSensor.isPressed())
            telemetry.addLine("Touch Sensor Pressed");
        else
            telemetry.addLine("Touch Sensor Released");




        telemetry.addLine(" ");
        telemetry.addData("raw ultrasonic", robot.rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", robot.rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", robot.rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", robot.rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.update();*/
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
