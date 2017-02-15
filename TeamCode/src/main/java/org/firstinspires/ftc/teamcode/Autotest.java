package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DragonoidsAuto;

@Disabled
//@Autonomous(name="Auto Test Blue", group="Blue")
public class Autotest extends DragonoidsAuto {

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
            if (detectColor()==1) {
                forward(-.05,.5);
                strafe(.6, 1);
            } else if (detectColor()==2){
                forward(.13, .4);
            }
            if (detectColor()==2) {
                forward(.05,.5);
                strafe(.6, 1);
            }
            strafe(-.5,1);
            forward(1.5,1);
            adjustRange();
            adjustHeading();
            alignLine(false);
            if (detectColor()==1) {
                forward(-.05,.5);
                strafe(.6, 1);
            } else if (detectColor()==2){
                forward(.13, .4);
            }
            if (detectColor()==2) {
                forward(.05,.5);
                strafe(.6, 1);
            }
            adjustRange();
            break;
       }
    }
}