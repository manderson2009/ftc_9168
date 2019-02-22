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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@TeleOp(name="Rover Ruckus Teleop", group="Thunderbot")
public class RoverRuckus_TeleOp extends OpMode{

    /* Declare OpMode members. */
    private RoverRuckusHardware robot       = new RoverRuckusHardware(); // use the class created to define a Pushbot's hardware


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
    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

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

        double crawl_speed = 2.0; //normal speed is half maximum

        if(crawlMode)
        {
            crawl_speed = 4.0;
        }
        robot.leftFront.setPower(left_front/magnitude/crawl_speed);
        robot.rightFront.setPower(right_front/magnitude/crawl_speed);
        robot.leftRear.setPower(left_rear/magnitude/crawl_speed);
        robot.rightRear.setPower(right_rear/magnitude/crawl_speed);

        if(gamepad1.dpad_up)
        {
            robot.lifter.setPower(.5);
        }
        else if(gamepad1.dpad_down)
        {
            robot.lifter.setPower(-.5);
        }
        else
        {
            robot.lifter.setPower(0);
        }



        telemetry.addData("drive Motor position: ", robot.leftFront.getCurrentPosition());
        telemetry.addData("Lifter Motor position: ", robot.lifter.getCurrentPosition());
        telemetry.addData("gyro reading: ",robot.gyro.getHeading());
        telemetry.addData("Color sensor red: ", robot.colorSensor.red());
        telemetry.addData("Color sensor blue: ", robot.colorSensor.blue());
        telemetry.addData("Color sensor green: ", robot.colorSensor.green());


//        telemetry.addData("rx: ",gamepad1.right_stick_x);
//        telemetry.addData("ry: ",gamepad1.right_stick_y);
//        telemetry.addData("lx: ",gamepad1.left_stick_x);
//        telemetry.addData("ly: ",gamepad1.left_stick_y);
//
//        telemetry.addData("leftFront motor: ",robot.leftFront.getPower());
//        telemetry.addData("rightFront motor: ",robot.rightFront.getPower());
//        telemetry.addData("leftRear motor: ",robot.leftRear.getPower());
//        telemetry.addData("rightRear motor: ",robot.rightRear.getPower());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
