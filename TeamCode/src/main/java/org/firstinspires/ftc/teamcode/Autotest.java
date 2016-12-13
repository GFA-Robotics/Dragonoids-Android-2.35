package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Dragonoids on 12/2/2016.
 */

@Autonomous(name="Auto Test", group="")
public class Autotest extends DragonoidsAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        //currently testing blue side full autonomous

        shoot();
        forward(-.7, .6);

        strafe(.6, .75);
        rightDiagonal(-2, .75);
        adjustRange();
        adjustHeading();
        alignLine(true);
        if (detectColor()) {
            strafe(.3,.5);
        } else {
            forward(-.15,.2);
            strafe(.3,.5);
        }
        strafe(-.3,1);
        forward(1.5,1);
        alignLine(false);
        if (detectColor()) {
            strafe(.3,.5);
        } else {
            forward(-.15,.2);
            strafe(.3,.5);
        }
        strafe(-.5,1);
/*
        //buttonPress(detectColor());
        strafe(-.2,.2);
        sleep(500);
        strafe(.5,.5);
        forward(-1.75,.75);
        alignLine(true);
        adjustHeading();
        strafe(-.3,.5);
        //buttonPress(detectColor());
        strafe(-.2,.2);

        leftDiagonal(2,.75);

*/



    }


}
