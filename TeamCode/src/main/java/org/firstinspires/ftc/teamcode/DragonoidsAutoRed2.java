package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Dragonoids on 2/3/2017.
 */

@Autonomous(name="Red Autonomous Cap Ball", group="Blue")
public class DragonoidsAutoRed2 extends DragonoidsAuto {

    //Auto Red 2 is a RED Side autonomous from Position 2 that performs: SHOOT, CAP BALL MOVE
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        while (opModeIsActive()) {
            sleep(8000);
            forward(-.4,.5);
            turn(40);
            shoot();
            forward(-.5,.6);
            strafe(.25,.5);
            sleep(500);
            strafe(-.2,.5);
        }

    }
}
