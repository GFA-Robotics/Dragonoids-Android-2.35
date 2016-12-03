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

        resetEncoders();

        forward(-.5,.5);
        shoot();
        sleep(2000);
        strafe(-1.5,1);


    }


}
