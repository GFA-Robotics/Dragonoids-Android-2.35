package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Dragonoids on 12/2/2016.
 */

@Disabled
public class Autotest extends DragonoidsAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        //currently testing blue side full autonomous
        shoot();
        forward(-.5, 1);

        turn(-120, 1);
        forward(2.15, 1);
        turn(-30, .5);
        telemetry.addData("Gyro Heading", gyro.getIntegratedZValue());
        telemetry.update();
        targetAngle = -180;
        adjustRange();
        adjustRange();
        adjustRange();
        adjustHeading();
        alignLine(true);
//        //false is red true is blue
        if (!detectColor()) {
            strafe(.5, 1);
        } else {
            forward(.13,.4);
            strafe(.5, 1);
        }
        strafe(-.8,1);
        targetAngle = -180;
        adjustHeading();
        forward(1.5, 1);
        adjustRange();
        adjustRange();
        adjustRange();
        alignLine(false);
        if (!detectColor()) {
            strafe(.5, 1);
        } else {
            forward(.13,.4);
            strafe(.5, 1);
        }
        strafe(-1.25,1);
        forward(-.5,1);
        turn(90,1);

    }


}
