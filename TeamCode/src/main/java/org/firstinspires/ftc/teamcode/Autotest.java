package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DragonoidsAuto;


@Autonomous(name="Auto Test", group="")
public class Autotest extends DragonoidsAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while (opModeIsActive()) {
            //red side full autonomous
            forward(-.75,.4);
            shoot();
            forward(-.7,.6);
            strafe(-.75, .6);
            turn(-180);
            leftDiagonal(.75, .75);
            adjustRange();
            adjustHeading();
            alignLine(true);
            if (detectColor()==1) {
                forward(-.05,.5);
                strafe(.5, 1);
            } else if (detectColor()==2){
                forward(.35, .4);
            }
            if (detectColor()==2) {
                forward(.05,.5);
                strafe(.5, 1);
            }
            strafe(-.5,1);
            forward(1.5,.75);
            adjustHeading();
            adjustRange();
            alignLine(false);
            if (detectColor()==1) {
                forward(-.05,.5);
                strafe(.5, 1);
            } else if (detectColor()==2){
                forward(.35, .4);
            }
            if (detectColor()==2) {
                forward(.05,.5);
                strafe(.5, 1);
            }
            adjustRange();
            //turn(0);

            break;
       }
    }
}