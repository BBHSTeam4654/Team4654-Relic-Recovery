package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "P mode forwards", group = "PMode")
public class PoPModeForwards extends PoPMode {
    @Override
    public void runOpMode() throws InterruptedException{
        super.forwards = true;
        super.runOpMode();

    }

}
