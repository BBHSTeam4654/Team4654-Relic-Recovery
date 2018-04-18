package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DodgyForwardsBlueBlock", group="dodgy")
public class DodgyForwardsBlueAutoOpBlock extends DodgyAutoOp {

    public DodgyForwardsBlueAutoOpBlock() {
        super(ColorMode.BLUE, DriveMode.FORWARDS, DropBlock.YES);
    }
    
}
