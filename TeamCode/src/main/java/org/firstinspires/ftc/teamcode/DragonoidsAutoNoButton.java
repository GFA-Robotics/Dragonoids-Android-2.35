package org.firstinspires.ftc.teamcode;

/**
 * Created by Dragonoids on 11/18/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous Shoot", group="Blue")
public class DragonoidsAutoNoButton extends DragonoidsAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        //get in position to shoot and shoot twice
        forward(-.7,.6);
        shoot();

        telemetry.addData("Distance Traveled: ", motorLF.getCurrentPosition() * (WHEEL_CIRC / ENCODER_CPR));
        telemetry.update();


    }
}
