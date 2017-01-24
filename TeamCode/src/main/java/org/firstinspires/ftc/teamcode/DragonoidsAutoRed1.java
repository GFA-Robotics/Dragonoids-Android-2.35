package org.firstinspires.ftc.teamcode;

/**
 * Created by Dragonoids on 11/18/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Red Autonomous Button", group="Red")
public class DragonoidsAutoRed1 extends DragonoidsAuto {


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while (opModeIsActive()) {
            //red side full autonomous
            forward(-.93,.6);
            shoot();
            forward(-.5,.6);
            strafe(-.5, .6);
            sleep(250);
            strafe(-.25, .8);
            turn(-180);
            leftDiagonal(1.75, .75);
            adjustHeading();
            adjustRange();
            alignLine(false);
            if (!detectColor()) {
                strafe(.6, 1);
            } else {
                forward(.13, .4);
                strafe(.6, 1);
            }
            strafe(-.5,1);
            forward(-1.5,1);
            adjustRange();
            adjustHeading();
            alignLine(true);
            if (!detectColor()) {
                strafe(.6, 1);
            } else {
                forward(.13, .4);
                strafe(.6, 1);
            }
            adjustRange();
            //turn(0);

            break;
        }
    }
}
