package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "P mode backwards", group = "PMode")
public class PoPModeBack extends PoPMode {
    @Override
    public void runOpMode() throws InterruptedException{
        super.forwards = false;
        super.distance = -1680;

        super.runOpMode();

    }
}
