package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="DodgyBackwardsBlueBlock", group="dodgy")
public class DodgyBackwardsBlueAutoOpBlock extends DodgyAutoOp {

    public DodgyBackwardsBlueAutoOpBlock() {
        super(ColorMode.BLUE, DriveMode.BACKWARDS, DropBlock.YES);
    }
    
}