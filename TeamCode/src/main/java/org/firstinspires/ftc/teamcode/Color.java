package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="Color")
public class Color extends OpMode {

	public ColorSensor color;
	public Servo servo;
	
	public void init() {
		color = hardwareMap.colorSensor.get("colorSensor");
		servo = hardwareMap.servo.get("arm");
	}
	
	public void loop() {
		servo.setPosition(gamepad1.left_trigger);
		telemetry.addData("pos", servo.getPosition());
		telemetry.addData("Color", color.red() > color.blue() ? "Red" : "Blue");
	}
}