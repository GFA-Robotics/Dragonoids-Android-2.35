package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Dragonoids on 2/27/2017.
 */
@Disabled
//@Autonomous(name="Red Autonomous Button Secondary", group="Red")
public class DragonoidsAutoRed3 extends DragonoidsAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while (opModeIsActive()) {
            forward(1.45,.6);
            strafe(1, .8);
            forward(2,1);
            adjustHeading();
            adjustRange();
            alignLine(false);
            if (detectColor()==1) {
                forward(-.05,.5);
                sleep(250);
                strafe(.75, 1);
            } else if (detectColor()==2){
                forward(.15, .4);
                sleep(250);
                strafe(.75, 1);
            }
            adjustRange();
            forward(-1.7,.5);
            adjustRange();
            adjustHeading();
            alignLine(true);
            if (detectColor()==1) {
                forward(-.05,.5);
                sleep(250);
                strafe(.75, 1);
            } else if (detectColor()==2){
                forward(.15, .4);
                sleep(250);
                strafe(.75, 1);
            }
            adjustRange();
            turn(90);
            shoot();
            strafe(-.5,1);
            forward(-1.55,.8);

            break;
        }
    }
}
