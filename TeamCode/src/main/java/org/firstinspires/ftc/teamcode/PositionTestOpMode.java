package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Position Test", group="Test")
public class PositionTestOpMode extends OpMode {

    private Servo claw;
    private Servo clawArm;

    public void init() {
        claw = hardwareMap.servo.get("claw");
        clawArm = hardwareMap.servo.get("clawArm");
    }
    
    public void loop() {
        claw.setPosition(gamepad1.left_trigger);
        clawArm.setPosition(gamepad1.right_trigger);
        
        telemetry.addData("Claw Pos", claw.getPosition());
        telemetry.addData("Claw Arm Pos", clawArm.getPosition());
    }
    
}