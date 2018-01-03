package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Test", group="Iterative Opmode")
public class DemoOpMode extends OpMode
{
	private CRServo claw;
	private ColorSensor color;

	/*
	 * Code to run ONCE when the driver hits INIT
	 */
	@Override
	public void init() {
		claw = hardwareMap.crservo.get("claw");
		color = hardwareMap.colorSensor.get("color");
	}

	/*
	 * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
	 */
	@Override
	public void loop() {
		// claw.setPosition((System.currentTimeMillis() / 2000) % 2);
		float f = gamepad1.right_trigger;
		claw.setPower(0.1f * (gamepad1.left_stick_y));
		
		telemetry.addData("Power", claw.getPower());
		telemetry.addData("Color", color.argb() & 0b11111111);
	}
	
	
	
	
	
	

}
