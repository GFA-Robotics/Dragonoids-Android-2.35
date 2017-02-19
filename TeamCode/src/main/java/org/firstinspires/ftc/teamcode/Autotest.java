package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DragonoidsAuto;

@Disabled
//@Autonomous(name="Auto Test", group="")
public class Autotest extends DragonoidsAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while (opModeIsActive()) {
            //blue side full autonomous
            forward(-.75, .4);
            shoot();
            forward(-.7, .6);
            strafe(.75, .6);
            rightDiagonal(-1, .75);
            adjustRange();
            adjustHeading();
            alignLine(false);
            if (detectColor()==2) {
                forward(-.05,.5);
                sleep(250);
                strafe(.5, 1);
            } else if (detectColor()==1){
                forward(.15, .4);
                sleep(250);
                strafe(.5, 1);
            }
            strafe(-.65,1);
            forward(-1.5,1);
            adjustRange();
            adjustHeading();
            alignLine(true);
            if (detectColor()==2) {
                forward(-.05,.5);
                sleep(250);
                strafe(.5, 1);
            } else if (detectColor()==1){
                forward(.15, .4);
                sleep(250);
                strafe(.5, 1);
            }
            adjustRange();
            break;
       }
    }
}