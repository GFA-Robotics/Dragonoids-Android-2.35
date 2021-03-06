package org.firstinspires.ftc.teamcode;

/**
 * Created by Dragonoids on 11/18/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@Autonomous(name="Blue Autonomous Button Old", group="Blue")
public class DragonoidsAutoBlueOld extends DragonoidsAuto {
//Auto Blue 1 is a BLUE Side autonomous from Position 1 that performs: SHOOT, CAP BALL MOVE, 2 BEACON CLAIMS
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while (opModeIsActive()) {
            //blue side full autonomous
            forward(-.25, .75);
            shoot();
            forward(-1.2, .6);
            strafe(.75, .6);
            rightDiagonal(-1, .75);
            adjustRange();
            adjustHeading();
            alignLine(false);
            if (detectColor()==2) {
                forward(-.05,.5);
                sleep(250);
                strafe(.75, 1);
            } else if (detectColor()==1){
                forward(.15, .4);
                sleep(250);
                strafe(.75, 1);
            }
            strafe(-.65,1);
            forward(-1.5,1);
            adjustRange();
            adjustHeading();
            alignLine(true);
            sleep(500);
            if (detectColor()==2) {
                forward(-.05,.5);
                sleep(250);
                strafe(.75, 1);
            } else if (detectColor()==1){
                forward(.15, .4);
                sleep(250);
                strafe(.75, 1);
            }
            adjustRange();
            break;
        }
    }
}