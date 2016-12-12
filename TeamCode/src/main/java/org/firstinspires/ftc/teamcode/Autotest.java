package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Dragonoids on 12/2/2016.
 */

@Autonomous(name="Auto Test", group="")
public class Autotest extends DragonoidsAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        //currently testing blue side full autonomous
        forward(-.7,.6);
        shoot();

        rightDiagonal(-2,.75);
        alignLine();
        strafe(-.3,.4);

        adjustHeading();
        //buttonPress(detectColor());
        strafe(-.2,.2);




    }


}
