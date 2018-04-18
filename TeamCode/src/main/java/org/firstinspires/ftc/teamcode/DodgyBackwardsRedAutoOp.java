package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="DodgyBackwardsRedAutoOp", group="dodgy")
public class DodgyBackwardsRedAutoOp extends DodgyAutoOp {

    public DodgyBackwardsRedAutoOp() {
        super(ColorMode.RED, DriveMode.BACKWARDS, DropBlock.NO);
    }

}