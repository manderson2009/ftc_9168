package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Park In Depot",group="Autonomous")
public class ParkInDepot extends LandAndSampleMinerals {

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
                robot.driveForward(-.1);
                sleep(2000);
                robot.motorsOff();
                sleep(500);

                break;

            case RIGHT_MINERAL:
                robot.driveForward(-.1);
                sleep(1000);
                robot.rotateLeft(.1);
                sleep(750);
                robot.driveForward(-0.1);
                sleep(2500);
                robot.motorsOff();
                sleep(500);

                break;

            case LEFT_MINERAL:
                robot.driveForward(-.1);
                sleep(1000);
                robot.rotateRight(.1);
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

        switch(mineralSampled)
        {
            case CENTER_MINERAL:

                robot.rotateRight(.1);
                sleep(1700);
                robot.motorsOff();
                robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sleep(500);



                driveForwardInches(27,.2);

                robot.motorsOff();
                robot.rotateLeft(.1);
                sleep(800);
                robot.motorsOff();
                driveForwardInches(51,.2);
                break;

            case RIGHT_MINERAL:
                robot.driveForward(.1);
                sleep(500);

                robot.rotateLeft(.1);
                sleep(1900);

                robot.motorsOff();

                driveForwardInches(87,-.2);


                break;

            case LEFT_MINERAL:
                robot.rotateRight(.1);
                sleep(500);
                robot.motorsOff();
                sleep(500);


                driveForwardInches(76,.2);

                break;
        }
    }

}
