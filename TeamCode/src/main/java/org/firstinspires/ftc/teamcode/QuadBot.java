package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class QuadBot extends OpMode {
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    public void init() {
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
    }
    
    public void loop() {
        double angle =Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + 3 * Math.PI / 4;
        double m = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double p1 = m * Math.sin(angle);
        double p2 = m * Math.cos(angle);
        double r = gamepad1.right_stick_x;
        
        rightBack.setPower(p1 + r);
        rightFront.setPower(r + p2);
        leftBack.setPower(r - p2);
        leftFront.setPower(r - p1);
        
        
        
        
    }
    
    
    
    
    
    
    
    
    
    
    
}