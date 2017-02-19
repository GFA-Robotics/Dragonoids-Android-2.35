package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Dragonoids on 2/3/2017.
 */

@Autonomous(name="Red Autonomous Cap Ball", group="Red")
public class DragonoidsAutoRed2 extends DragonoidsAuto {

    //Auto Red 2 is a RED Side autonomous from Position 2 that performs: SHOOT, CAP BALL MOVE
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while (opModeIsActive()) {
            sleep(10000);
            forward(-1.6,.5);
            shoot();
            forward(-1.2,.6);
            sleep(500);
            forward(-.25,.6);
            break;
        }

    }
}
