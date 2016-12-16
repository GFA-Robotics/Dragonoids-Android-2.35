package org.firstinspires.ftc.teamcode;

/**
 * Created by Dragonoids on 11/18/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Blue Autonomous Button", group="Blue")
public class DragonoidsAutoBlue1 extends DragonoidsAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        //currently testing blue side full autonomous
        shoot();
        forward(-.5, 1);

        turn(-25, 1);
        forward(-2.4, 1);
        turn(25,.5);
        adjustRange();
        adjustRange();
        adjustHeading();
        alignLine(false);
//        //false is red true is blue
        if (detectColor()) {
            strafe(.5, 1);
        } else {
            forward(.13,.4);
            strafe(.5, 1);
        }
        strafe(-.85,1);
        adjustHeading();
        forward(-1.5, 1);
        adjustRange();
        adjustRange();
        adjustHeading();
        alignLine(true);
        if (detectColor()) {
            strafe(.5, 1);
        } else {
            forward(.13,.4);
            strafe(.5, 1);
        }
        strafe(-1.25,1);
        forward(.5,1);
        turn(-90,1);
    }
}
