package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
//420 pulses per rotation
@Disabled
public class EncoderTest extends TrustyAutoOp {

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 
        int travelled = 0;
        int old = rightBack.getCurrentPosition();
        while(travelled < 1604){
         setPowers(1,1,1,1);
         travelled += rightBack.getCurrentPosition()-old; 
         old = rightBack.getCurrentPosition();
        }
    }
}