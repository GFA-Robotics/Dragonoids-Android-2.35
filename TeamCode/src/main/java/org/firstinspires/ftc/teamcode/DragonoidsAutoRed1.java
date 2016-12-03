package org.firstinspires.ftc.teamcode;

/**
 * Created by Dragonoids on 11/18/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Red Autonomous 1", group="Red")
public class DragonoidsAutoRed1 extends DragonoidsAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        //get in position to shoot and shoot twice
        forward(-.5,.6);
        shoot();

        //move in pos to detect left light of first beacon
        strafe(-1,1);
        forward(-1.35,.6);
        turn(180,1);
        strafe(1.66, 1);

        buttonPress(detectColor());
        sleep(750);
        strafe(.4,1);
        sleep(1500);
        strafe(-1,1);
        buttonPresser.setPosition(.5);

        forward(2.15,.6);
        strafe(-.8,1);

        buttonPress(detectColor());
        sleep(750);
        strafe(-.3,1);
        sleep(1500);
        strafe(1,1);
        buttonPresser.setPosition(.5);

        forward(-2,1);

        telemetry.addData("Distance Traveled: ", motorLF.getCurrentPosition() * (WHEEL_CIRC / ENCODER_CPR));
        telemetry.update();


    }
}
