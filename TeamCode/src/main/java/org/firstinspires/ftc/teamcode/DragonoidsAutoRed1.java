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

        while (opModeIsActive()) {
            forward(1.5,.6);
            strafe(1.25, .8);
            forward(2.15,1);
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
            forward(-1.7,.65);
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
            strafe(-.9,.8);
            turn(100);
            shoot();
            strafe(-.65,1);
            forward(-1.45,.8);

            break;
        }
    }
}
