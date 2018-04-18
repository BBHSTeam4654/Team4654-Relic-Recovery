package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class TrustyAutoOp extends LinearBaseOpMode {

    protected static enum ColorMode {
        BLUE, RED;
    }
    
    ColorMode colorMode;
    
    public TrustyAutoOp(ColorMode colorMode) {
        this.colorMode = colorMode;
    }
    
    public TrustyAutoOp() {
        this(null);
    }
    
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        
        colorArm.setPosition(0.6);
        waitForStart();

        colorArm.setPosition(.06);
        sleep(1000);
        
        int red = color.red();
        int blue = color.blue();
        telemetry.log().add(blue > red ? "Blue" : "Red");
        if ((blue > red) == (colorMode == ColorMode.BLUE)) {
            setPowers(0.2, 0.2, 0.2, 0.2);
        } else {
            setPowers(-0.2, -0.2, -0.2, -0.2);
        }
        sleep(500);
        setPowers(0, 0, 0, 0);
        colorArm.setPosition(0.6);
        sleep(1000);
    }

}