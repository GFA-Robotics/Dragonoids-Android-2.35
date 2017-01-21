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

        while (opModeIsActive()) {
            //blue side full autonomous
            forward(-.93,.6);
            shoot();
            forward(-.5,.6);
            strafe(.5, .6);
            sleep(250);
            strafe(.25, .8);
            rightDiagonal(-1.4, .75);
            adjustHeading();
            adjustRange();
            alignLine(true);
            if (detectColor()) {
                strafe(.6, 1);
            } else {
                forward(.13, .4);
                strafe(.6, 1);
            }
            strafe(-.5,1);
            forward(1.5,1);
            adjustRange();
            adjustHeading();
            alignLine(false);
            if (detectColor()) {
                strafe(.5, 1);
            } else {
                forward(.13, .4);
                strafe(.5, 1);
            }
            adjustRange();
            break;
        }
    }
}