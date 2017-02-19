package org.firstinspires.ftc.teamcode;

/**
 * Created by Dragonoids on 11/18/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Red Autonomous Button", group="Red")
public class DragonoidsAutoRed1 extends DragonoidsAuto {

    //Auto Red 1 is a RED Side autonomous from Position 1 that performs: SHOOT, CAP BALL MOVE, 2 BEACON CLAIMS
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while (opModeIsActive()) {
            //red side full autonomous
            forward(-.75, .75);
            shoot();
            forward(-.7, .6);
            strafe(-.75, .6);
            turn(-180);
            leftDiagonal(.75, .75);
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
            strafe(-.5,1);
            forward(1.5,.75);
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
            //turn(0);

            break;
        }
    }
}
