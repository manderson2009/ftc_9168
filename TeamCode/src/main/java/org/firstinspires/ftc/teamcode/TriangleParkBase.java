package org.firstinspires.ftc.teamcode;

/**
 * Created by ma61444 on 2/2/2018.
 */

public abstract class TriangleParkBase extends BaseAutoMode{

    public boolean movedLeft;

    public void InitAndKnockBall()
    {
    }

    public void updateTelemetry()
    {
        telemetry.addData("Color sensor red: ", robot.colorSensor.red());
        telemetry.addData("Color sensor blue: ", robot.colorSensor.blue());
        telemetry.addData("motor encoder ",robot.lifter.getCurrentPosition());
        telemetry.update();
    }
}
