package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="DodgyBackwardsBlueAutoOp", group="dodgy")
public class DodgyBackwardsBlueAutoOp extends DodgyAutoOp {

    public DodgyBackwardsBlueAutoOp() {
        super(ColorMode.BLUE, DriveMode.BACKWARDS, DropBlock.NO);
    }
    
}