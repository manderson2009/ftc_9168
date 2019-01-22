package org.firstinspires.ftc.teamcode;

public abstract class LandAndSampleMinerals extends BaseAutoMode {

    enum MineralPositionSampled {LEFT_MINERAL,CENTER_MINERAL,RIGHT_MINERAL};

    // Adjust these values for readings taken in playing field
    final int BLOCK_DETECTED_RED_VALUE = 40;
    final double BLOCK_DETECTION_BLUE_RED_RATIO = 0.85;

    public void initAndLand() {
        //Robot will wait until PLAY is pressed on phone app
        InitLoopWaitForStartButton();

        // Lower Robot
        {
            // Lift up lifter (to go down)
            robot.lifter.setPower(0.1);

            // Wait time for robot to lower
            sleep(5850);

            // shut off lifter
            robot.lifter.setPower(0);
        }

        sleep(500);

        // Drive Right to unlatch
        {
            // Set motors to crab right
            robot.crabRight(.1);

            // Wait time for robot to move
            sleep(500);

            robot.motorsOff();
        }
        sleep(500);
    }

    public boolean navigateToMinerals() {
        // Reverse to mineral area
        robot.driveForward(-.1);
        sleep(1600);
        robot.motorsOff();
        sleep(500);


        // Crawl right until color changes
        return crawlLateralToBlock();
    }

    public MineralPositionSampled sampleMinerals(){
        //Sample first block, if yellow hit it
        if(blockIsYellow())
        {
            //hit the block
            robot.crabLeft(.1);
            sleep(500);
            robot.motorsOff();
            robot.driveForward(-.1);
            sleep(500);
            robot.motorsOff();
            sleep(500);


            return MineralPositionSampled.CENTER_MINERAL;
        }

        robot.crabRight(.1);
        sleep(750);

        if(!crawlLateralToBlock())
        {
            // Couldn't find the block, hit whatever is there and move on.
            robot.driveForward(-.1);
            sleep(500);
            robot.motorsOff();
            return MineralPositionSampled.RIGHT_MINERAL;
        }

        //Sample block, if yellow hit it
        if(blockIsYellow())
        {
            robot.driveForward(-.1);
            sleep(500);
            robot.motorsOff();
            sleep(500);
            //hit the block
            return MineralPositionSampled.RIGHT_MINERAL;
        }

        // Move to left mineral based on time.

        robot.driveForward(.1);
        sleep(500);
        robot.motorsOff();
        sleep(500);

        robot.crabLeft(.1);
        sleep(4500);
        robot.motorsOff();
        sleep(500);

        robot.driveForward(-.1);
        sleep(1000);
        robot.motorsOff();
        sleep(500);

        //Block is yellow
        //hit the block
        return MineralPositionSampled.LEFT_MINERAL;
    }

    public boolean crawlLateralToBlock()
    {
        boolean ballFound = false;
        int i =0;
        robot.crabRight(.1);
        while(i<40)
        {
            if(robot.colorSensor.red() > BLOCK_DETECTED_RED_VALUE)
            {
                ballFound = true; // block/ball found
                break;
            }
            i++;
            sleep(50);
        }
        robot.motorsOff();
        sleep(500);

        return ballFound;
    }

    public boolean blockIsYellow() {
        // Notes on sensor reading
        // RGB values
        // yellow block: 100,90,61
        // White ball: 116, 130, 110
        // compare ratio to blue/red to determine color
        double red_scaled = robot.colorSensor.red();
        double blue_scaled = robot.colorSensor.blue();

        if(blue_scaled/red_scaled < BLOCK_DETECTION_BLUE_RED_RATIO)
        {
            return true;//Yellow block detected
        }
        else
        {
            return false;
        }
    }
}
