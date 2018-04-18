package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DodgyForwardsBlueAutoOp", group="dodgy")
public class DodgyForwardsBlueAutoOp extends DodgyAutoOp {

    public DodgyForwardsBlueAutoOp() {
        super(ColorMode.BLUE, DriveMode.FORWARDS, DropBlock.NO);
    }
    
}
