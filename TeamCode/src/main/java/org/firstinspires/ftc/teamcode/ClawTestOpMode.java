package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Demo!", group="Iterative Opmode")
@Disabled
public class ClawTestOpMode extends OpMode
{
	
	enum State {
		DRIVE,
		MECANUM
	}

	private State state = State.DRIVE;	
	private CRServo claw;
	private ColorSensor color;
	private DcMotor leftFront, rightFront, leftBack, rightBack, armPitch, armLength;

	@Override
	public void init() {
		claw = hardwareMap.crservo.get("claw");
		color = hardwareMap.colorSensor.get("colorSensor");
		leftFront = hardwareMap.dcMotor.get("leftFront");
		rightFront = hardwareMap.dcMotor.get("rightFront");
		leftBack = hardwareMap.dcMotor.get("leftBack");
		rightBack = hardwareMap.dcMotor.get("rightBack");
		armPitch = hardwareMap.dcMotor.get("armPitch");
		armLength = hardwareMap.dcMotor.get("armLength");
		
		rightFront.setDirection(DcMotor.Direction.REVERSE);
		rightBack.setDirection(DcMotor.Direction.REVERSE);
	}

	@Override
	public void loop() {
		float f = gamepad1.right_trigger;
		claw.setPower(0.1f * (gamepad1.right_stick_y));
		//Right stick x is arm pitch x is extend y for retract
		armPitch.setPower(1f * gamepad1.right_stick_x);
		
		armLength.setPower(.25f * (gamepad1.x ? 1 : (gamepad1.y ? -1 : 0)));
		if (gamepad1.dpad_up)
			state = State.DRIVE;
		else if (gamepad1.dpad_right)
			state = State.MECANUM;
		
		float x = gamepad1.left_stick_x, y = gamepad1.left_stick_y;
		switch (state) {
			case DRIVE:
				setPowers(y - x, y - x, y + x, y + x);
				break;
			case MECANUM:
				double power = Math.sqrt(x * x + y * y);
				double angle = Math.atan2(y, x);
				double sin = Math.sin(angle + Math.PI / 4);
				double cos = Math.cos(angle + Math.PI / 4);

				setPowers(power * sin, power * cos, power * cos, power * sin);
				break;
		}
		
		telemetry.addData("State", state);
		telemetry.addData("Power", claw.getPower());
		telemetry.addData("Color", color.argb() & 0b11111111);
		telemetry.addData("Arm Pitch Power", armPitch.getPower());
	}
	
	private void setPowers(double leftFront, double leftBack, double rightFront, double rightBack) {
		this.leftFront.setPower(leftFront);
		this.leftBack.setPower(leftBack);
		this.rightFront.setPower(rightFront);
		this.rightBack.setPower(rightBack);
	}
}