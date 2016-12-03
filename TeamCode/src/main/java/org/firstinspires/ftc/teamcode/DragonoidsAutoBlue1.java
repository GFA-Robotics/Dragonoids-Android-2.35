package org.firstinspires.ftc.teamcode;

/**
 * Created by Dragonoids on 11/18/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Blue Autonomous 1", group="Blue")
public class DragonoidsAutoBlue1 extends DragonoidsAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        //get in position to shoot and shoot twice
        forward(-.5,.6);
        shoot();

        //move in pos to detect left light of first beacon
        strafe(1,.75);
        forward(-1.35,.6);
        strafe(1.66, .75);

        buttonPress(detectColor());
        sleep(750);
        strafe(.4,.75);
        sleep(1500);
        strafe(-1,.75);
        buttonPresser.setPosition(.5);

        forward(-2,.6);
        strafe(.8,.75);

        buttonPress(detectColor());
        sleep(750);
        strafe(.3,.75);
        sleep(1500);
        strafe(-1,.75);
        buttonPresser.setPosition(.5);

        forward(2,1);

        telemetry.addData("Distance Traveled: ", motorLF.getCurrentPosition() * (WHEEL_CIRC / ENCODER_CPR));
        telemetry.update();


    }
}
