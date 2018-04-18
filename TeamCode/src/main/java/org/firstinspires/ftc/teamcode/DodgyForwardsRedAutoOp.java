package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DodgyForwardsRedAutoOp", group="dodgy")
public class DodgyForwardsRedAutoOp extends DodgyAutoOp {

    public DodgyForwardsRedAutoOp() {
        super(ColorMode.RED, DriveMode.FORWARDS, DropBlock.NO);
    }
}
