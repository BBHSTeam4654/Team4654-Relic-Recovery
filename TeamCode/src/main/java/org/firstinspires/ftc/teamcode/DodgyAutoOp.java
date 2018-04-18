package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DodgyAutoOp extends LinearBaseOpMode {

    protected static enum ColorMode {
        BLUE, RED;
    }
    
    protected static enum DriveMode {
        FORWARDS, BACKWARDS;
    }
    
    protected static enum DropBlock {
        YES, NO;
    }
    
    ColorMode colorMode;
    DriveMode driveMode;
    DropBlock blockMode;
    
    int inches;
    
    public DodgyAutoOp(ColorMode colorMode, DriveMode driveMode, DropBlock blockMode, int inches) {
        this.colorMode = colorMode;
        this.driveMode = driveMode;
        this.blockMode = blockMode;
        this.inches = inches;
    }
    
    public DodgyAutoOp(ColorMode colorMode, DriveMode driveMode, DropBlock blockMode) {
        this(colorMode, driveMode, blockMode, 91);
    }
    
    public DodgyAutoOp() {
        this(null, null, null);
    }
    
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        glyph.setPosition(0.4);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER); 
        int start = rightBack.getCurrentPosition();
        
        // Number of pulses to travel 3 feet: 36" * 420 ppr / 4pi in/rev
        final int end = (int) Math.round(inches * 420 / (4 * Math.PI));
        
        colorArm.setPosition(0.6);
        waitForStart();

        colorArm.setPosition(.064);
        sleep(1000);
        
        int red = color.red();
        int blue = color.blue();
        telemetry.log().add(blue > red ? "Blue" : "Red");
        if ((blue > red) == (colorMode == ColorMode.BLUE)) {
            setPowers(0.2, 0.2, 0.2, 0.2);
        } else {
            setPowers(-0.2, -0.2, -0.2, -0.2);
        }
        sleep(672);
        setPowers(0, 0, 0, 0);
        colorArm.setPosition(0.6);
        // colorArm.setPosition(0.85);
        sleep(1000);
        
        double dir = driveMode == DriveMode.FORWARDS ? 1 : (driveMode == DriveMode.BACKWARDS ? -1 : 0);
        setPowers(dir, dir, dir, dir);
        
        
        
        while (Math.abs(rightBack.getCurrentPosition() - start) < end) {
            telemetry.addData("Distance", String.format("%d/%d", Math.abs(rightBack.getCurrentPosition() - start), end));
        }
        
        colorArm.setPosition(0.6);
        setPowers(0, 0, 0, 0);
        sleep(1000);
        
        if (blockMode == DropBlock.YES) {
            setPowers(-0.2, 0.2, 0.2, -0.2);
            sleep(2000);
            setPowers(0, 0, 0, 0);
            sleep(750);
            glyph.setPosition(0);
            setPowers(0.2, -0.2, -0.2, 0.2);
            sleep(1000);
            setPowers(0, 0, 0, 0);
        } else {
            glyph.setPosition(0);
        }
    }
    
}