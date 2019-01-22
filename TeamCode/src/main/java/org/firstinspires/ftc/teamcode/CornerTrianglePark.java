package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by ma61444 on 2/2/2018.
 */

//@Autonomous(name="CornerTrianglePark",group="Autonomous")
public class CornerTrianglePark extends TriangleParkBase{
    @Override
    public void runOpMode() {
        InitAndKnockBall();

        robot.driveForward(.1);
        sleep(1000);
        robot.motorsOff();

        if((redTeam && movedLeft) || (!redTeam && !movedLeft))
        {
            robot.driveForward(.5);
            sleep(2000);
            robot.motorsOff();

            if(redTeam)
            {
                turnRight(100);
            }
            else
            {
                turnLeft(100);
            }
            sleep(100);

            robot.driveForward(.5);
            sleep(1750);
            robot.motorsOff();
        }
        else
        {
            robot.driveForward(.1);
            sleep(500);
            robot.motorsOff();

            if(redTeam)
            {
                robot.crabRight(.5);
            }
            else
            {
                robot.crabLeft(.5);
            }
            sleep(2000);
            robot.motorsOff();

            robot.driveForward(.5);
            sleep(750);
            robot.motorsOff();
        }

        robot.driveForward(.5);
        sleep(250);
        robot.motorsOff();


        sleep(2000);
    }
}
