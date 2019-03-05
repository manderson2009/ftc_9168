package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Park In Depot",group="Autonomous")
public class ParkInDepot extends LandAndSampleMinerals {

    @Override
    public void runOpMode() {

        initRobot();

        MineralPositionSampled mineral = sampleMinerals();

        LandRobot();

        navigateToMinerals(mineral);

        parkAndClaimDepot(mineral);


    }

    public void parkAndClaimDepot(MineralPositionSampled mineralSampled)    {

        // Park in depot (maybe change depending on where the mineral was)

        switch(mineralSampled)
        {
            case CENTER_MINERAL:
                robot.driveForward(-.1);
                sleep(2000);
                robot.motorsOff();
                sleep(500);
                break;

            case RIGHT_MINERAL:
                robot.driveForward(-.1);
                sleep(1000);
                robot.rotateRight(.1);
                sleep(750);
                robot.driveForward(-0.1);
                sleep(1500);
                robot.motorsOff();
                sleep(500);

                break;

            case LEFT_MINERAL:
                robot.driveForward(-.1);
                sleep(1000);
                robot.rotateLeft(.1);
                sleep(600);
                robot.driveForward(-0.1);
                sleep(1500);
                robot.motorsOff();
                sleep(500);
                break;
        }

        // Claim the depot
        robot.flipper.setPosition(1);
        sleep(1000);

    }

}
