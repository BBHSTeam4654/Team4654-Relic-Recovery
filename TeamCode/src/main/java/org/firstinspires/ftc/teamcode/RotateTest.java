package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by cyphecx on 1/29/18.
 */

@Autonomous(name="RotateTest")
public class RotateTest extends OpMode {

    private DcMotor leftBack, leftFront, rightBack, rightFront;

    @Override
    public void init() {
        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
    }

    @Override
    public void loop() {
        leftBack.setPower(.5);
        leftFront.setPower(.5);
        rightBack.setPower(.5);
        rightFront.setPower(.5);
    }
}
