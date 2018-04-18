package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="DodgyBackwardsRedBlock", group="dodgy")
public class DodgyBackwardsRedAutoOpBlock extends DodgyAutoOp {

    public DodgyBackwardsRedAutoOpBlock() {
        super(ColorMode.RED, DriveMode.BACKWARDS, DropBlock.YES);
    }

}