package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DodgyForwardsRedBlock", group="dodgy")
public class DodgyForwardsRedAutoOpBlock extends DodgyAutoOp {

    public DodgyForwardsRedAutoOpBlock() {
        super(ColorMode.RED, DriveMode.FORWARDS, DropBlock.YES);
    }
}
