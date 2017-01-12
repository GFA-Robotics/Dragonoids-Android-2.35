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
            shoot();

            forward(-.5, 1);

            turn(-120, 1);
            forward(2.5, 1);
            turn(-30, .5);
            telemetry.addData("Gyro Heading", gyro.getIntegratedZValue());
            telemetry.update();
            targetAngle = -180;
            adjustRange();
            adjustHeading();
            alignLine(true);
//        //false is red true is blue
            if (!detectColor()) {
                strafe(.5, 1);
            } else {
                forward(.13, .4);
                strafe(.5, 1);
            }
            strafe(-.8, 1);
            targetAngle = -180;
            // adjustHeading();
            forward(1.5, 1);
            adjustRange();
            alignLine(false);
            if (!detectColor()) {
                strafe(.5, 1);
            } else {
                forward(.13, .4);
                strafe(.5, 1);
            }
            strafe(-.75, 1);
            forward(-.75, 1);
            break;
        }
    }
}
