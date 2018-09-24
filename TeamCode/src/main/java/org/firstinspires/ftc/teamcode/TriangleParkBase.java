package org.firstinspires.ftc.teamcode;

/**
 * Created by ma61444 on 2/2/2018.
 */

public abstract class TriangleParkBase extends BaseAutoMode{

    public boolean movedLeft;

    public void InitAndKnockBall()
    {
        //Operator selects Color and presses start on controller
        //Robot will wait until PLAY is pressed on phone app
        InitLoopWaitForStartButton();


        robot.lowerBallServo();

        robot.driveForward(-.1);

        sleep(2200);

        robot.driveForward(0);
        updateTelemetry();

        sleep(100);
        if(robot.colorSensor.red() > 1 || robot.colorSensor.blue() > 1) {

            robot.driveForward(-.1);

            sleep(200);

            robot.driveForward(0);
            sleep(100);
        }


        if(robot.colorSensor.red() > 1 || robot.colorSensor.blue() > 1)
        {
            if(robot.colorSensor.red() > 1)
            {
                if(redTeam) {
                    robot.crabRight(.1);
                    sleep(1500);
                    robot.motorsOff();
                }
                else
                {
                    robot.crabLeft(.1);
                    sleep(1500);
                    robot.motorsOff();
                    movedLeft = true;
                }
            }
            else
            {
                if(redTeam) {
                    robot.crabLeft(.1);
                    sleep(1900);
                    robot.motorsOff();
                    movedLeft = true;
                }
                else
                {
                    robot.crabRight(.1);
                    sleep(1900);
                    robot.motorsOff();
                }
            }
        }
        //Stow ball servo
        robot.stowBallServo();
        sleep(250);
    }

    public void updateTelemetry()
    {
        telemetry.addData("Color sensor red: ", robot.colorSensor.red());
        telemetry.addData("Color sensor blue: ", robot.colorSensor.blue());
        telemetry.addData("motor encoder ",robot.lifter.getCurrentPosition());
        telemetry.update();
    }
}
