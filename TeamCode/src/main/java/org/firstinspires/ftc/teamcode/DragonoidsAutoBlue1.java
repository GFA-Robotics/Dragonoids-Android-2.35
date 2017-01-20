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
            //currently testing blue side full autonomous
            shoot();
            forward(-.5, 1);

            turn(-25);
            forward(-2.5, 1);
            turn(25);
            adjustRange();
            adjustHeading();
            alignLine(false);
            //false is red true is blue
            if (detectColor()) {
                strafe(.75, 1);
            } else {
                forward(.13, .4);
                strafe(.75, 1);
            }
            strafe(-.85, 1);
            adjustHeading();
            forward(-1.5, 1);
            adjustRange();
            alignLine(true);
            if (detectColor()) {
                strafe(.75, 1);
            } else {
                forward(.13, .4);
                strafe(.75, 1);
            }
            strafe(-.75, 1);
            forward(.5, 1);
            break;
        }
    }
}