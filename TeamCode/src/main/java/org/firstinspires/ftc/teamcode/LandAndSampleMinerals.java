package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class LandAndSampleMinerals extends BaseAutoMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AZivFcz/////AAABmaeRk9g0B0LylHj9TMS3G1cRzh+973U3DW/b2f4ir/BwjiGF/qsKHhr78/ICYxLbO87UyqNo3G+h5qEcAE2IfPgZeNMN/OKPunsdMG1htEQswf2cqEZib4lcZWQSfygM4bfHvDzQ1ngpLNRQi315vZGlb70iCSPCgOloIiQimw5zs4CWfJOilwXGMg03wrUBZ81TXL/YL3BEFfVuktliLdQ2C6BQ8W2HvYojJ40cULrZT75EK8rSSHtGMrM4u7BrTiQxYisieTi1av5zpXuF/GqSEsQGYEUdg2CPUrhV7JQMlw1gkfZCe4eDaDy1l6JzhoAGsQNoZKD+pU6ht48W/Elf5ioZF/zqvHVHpPWyZmr+\n";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    enum MineralPositionSampled {LEFT_MINERAL,
        CENTER_MINERAL,
        RIGHT_MINERAL, //From center of field viewpoint
        NO_MINERAL};

    // Adjust these values for readings taken in playing field
    final int BLOCK_DETECTED_RED_VALUE = 50;
    final double BLOCK_DETECTION_BLUE_RED_RATIO = 0.85;

    public void initRobot(){
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        //Robot will wait until PLAY is pressed on phone app
        InitLoopWaitForStartButton();


        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }


    }
    public void LandRobot() {

        // Lower Robot
        {
            // Reset encoders
            robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Lift up lifter (to go down)
            robot.lifter.setPower(0.2);

            // Wait time for robot to lower
            // TODO instead of waiting, use encoder position to
            //sleep(7080);
            while (robot.lifter.getCurrentPosition()<3300)
            {
                sleep(10);
            }

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

        driveForwardInches(10,-.1);
    }

    public void navigateToMinerals(MineralPositionSampled mineralSampled) {

        switch(mineralSampled)
        {
            case CENTER_MINERAL:

                // drive left 500 ms
                robot.crabLeft(.15);
                sleep(500);
                robot.motorsOff();
                sleep(500);

                //drive forward 1000ms
                driveForwardInches(15,-.1);
                break;

            case RIGHT_MINERAL:
                // drive left 500 ms
                robot.crabLeft(.15);
                sleep(1500);
                robot.motorsOff();
                sleep(500);

                //drive forward 1000ms
                driveForwardInches(15,-.1);
                break;

            case LEFT_MINERAL:

                // drive left 500 ms
                robot.crabRight(.15);
                sleep(1000);
                robot.motorsOff();
                sleep(500);

                //drive forward 1000ms
                driveForwardInches(15,-.1);


                break;
        }



    }

    public MineralPositionSampled sampleMinerals(){
        MineralPositionSampled sampledMineral = MineralPositionSampled.NO_MINERAL;

        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        while(sampledMineral == MineralPositionSampled.NO_MINERAL) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // We changed the size to 2, since we only take a picture of the left minerals
                if (updatedRecognitions.size() == 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }

                    // Check if there are two silver minerals
                    if (goldMineralX < 0) {
                        telemetry.addData("Gold Mineral Position", "Right");
                        sampledMineral = MineralPositionSampled.RIGHT_MINERAL;
                    } else if (goldMineralX < silverMineral1X) {
                        telemetry.addData("Gold Mineral Position", "Left");
                        sampledMineral = MineralPositionSampled.LEFT_MINERAL;
                    } else {
                        telemetry.addData("Gold Mineral Position", "Center");
                        sampledMineral = MineralPositionSampled.CENTER_MINERAL;
                    }


                }

                telemetry.update();
            }
        }
        return sampledMineral;
    }

    public void driveForwardInches(double inches, double power)
    {
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);

        robot.driveForward(.01);
        sleep(500);
        robot.driveForward(power/2);
        sleep(500);
        robot.driveForward(power);

        double multiplier = (0.000854 * inches) + 0.979;

        double reverse = 1;
        if(power < 0)
        {
            reverse = -1;
        }

        while (reverse * robot.leftFront.getCurrentPosition() < inches * robot.INCHES_TO_COUNTS_RATIO * multiplier)
        {
            sleep(10);
        }
        robot.motorsOff();
        sleep(500);
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
