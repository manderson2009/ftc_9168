package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Park In Crater",group="Autonomous")
public class ParkInCrater extends LandAndSampleMinerals {

    @Override
    public void runOpMode() {
        initRobot();


        LandRobot();

        MineralPositionSampled mineral = sampleMinerals();

        navigateToMinerals(mineral);

        parkAndClaimDepot(mineral);

    }

    public void parkAndClaimDepot(MineralPositionSampled mineralSampled)    {

        // Park in depot (maybe change depending on where the mineral was)

        switch(mineralSampled)
        {
            case CENTER_MINERAL:
                robot.driveForward(-.2);
                sleep(1000);
                robot.motorsOff();
                sleep(500);
                break;

            case RIGHT_MINERAL:
                robot.driveForward(-0.2);
                sleep(1000);
                robot.motorsOff();
                sleep(500);

                break;

            case LEFT_MINERAL:
                robot.driveForward(-0.2);
                sleep(1000);
                robot.motorsOff();
                sleep(500);
                break;
        }

    }

}
