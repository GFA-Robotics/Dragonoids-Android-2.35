package org.firstinspires.ftc.teamcode;

/**
 * Created by Dragonoids on 11/18/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
//@Autonomous(name="Shoot", group="")
public class DragonoidsAutoNoButton extends DragonoidsAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        //get in position to shoot and shoot twice
        sleep(10000);
        shoot();
    }
}
