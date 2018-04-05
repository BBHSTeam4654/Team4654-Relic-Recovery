package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="New Demo", group="Iterative Opmode")
public class NewDemoOpMode extends OpMode
{

	enum State {
		DRIVE,
		MECANUM,
		TANK,
		MECANUM2
	}

	private State state = State.DRIVE;
	private Servo colorArm;
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
		colorArm = hardwareMap.servo.get("arm");
		armPitch = hardwareMap.dcMotor.get("armPitch");
		armLength = hardwareMap.dcMotor.get("armLength");

		rightFront.setDirection(DcMotor.Direction.REVERSE);
		rightBack.setDirection(DcMotor.Direction.REVERSE);

		armPitch.setDirection(DcMotor.Direction.REVERSE);
		colorArm.setDirection(Servo.Direction.REVERSE);

	}

	@Override
	public void loop() {
		double armMult = gamepad2.left_bumper ? 0.6 : (gamepad2.right_bumper ? 0.3 : 1.0);
		double clawMult = gamepad2.left_bumper ? 0.8 : (gamepad2.right_bumper ? 0.6 : 1.0);
		claw.setPower(clawMult * 0.15f * (gamepad2.right_stick_y));
		//Right stick x is arm pitch x is extend y for retract
		armPitch.setPower(armMult * 1f * gamepad2.left_stick_y);

		//	armLength.setPower(.25f * (gamepad2.right_trigger  ? 1 : (gamepad2.left_trigger ? -1 : 0)));
		armLength.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
		if (gamepad1.dpad_up)
			state = State.DRIVE;
		else if (gamepad1.dpad_right)
			state = State.MECANUM;
		else if (gamepad1.dpad_down)
			state = State.TANK;
		else if (gamepad1.dpad_left)
			state = State.MECANUM2;

		if (gamepad1.x || gamepad2.x)
			colorArm.setPosition(0.8);
		if (gamepad1.y || gamepad2.y)
			colorArm.setPosition(0.3);

		double mult = gamepad1.left_bumper ? 0.5 : (gamepad1.right_bumper ? 0.2 : 1.0);
		double x = gamepad1.left_stick_x, y = gamepad1.left_stick_y;
		switch (state) {
			case DRIVE:
				setPowers(mult, y - x, y - x, y + x, y + x);
				break;
			case MECANUM:
				double power = Math.sqrt(x * x + y * y);
				double angle = Math.atan2(y, x);
				double sin = Math.sin(angle - Math.PI / 4);
				double cos = Math.cos(angle - Math.PI / 4);

				setPowers(mult, power * sin, power * cos, power * cos, power * sin);
				break;
			case MECANUM2:
				double power2 = Math.sqrt(x * x + y * y);
				double angle2 = Math.atan2(y, x);
				double sin2 = Math.sin(angle2 - Math.PI / 4);
				double cos2 = Math.cos(angle2 - Math.PI / 4);

				double turn = -gamepad1.right_stick_x;

				setPowers(mult, power2 * sin2 + turn, power2 * cos2 + turn, power2 * cos2 - turn, power2 * sin2 - turn);
				break;
			case TANK:
				//left is y
				double left = gamepad1.left_stick_y;
				double right = gamepad1.right_stick_y;
				setPowers(mult, left, left, right, right);
				break;
		}

		telemetry.addData("State", state);
		telemetry.addData("Power", claw.getPower());
		telemetry.addData("Color", color.argb() & 0b11111111);
	}

	private void setPowers(double mult, double leftFront, double leftBack, double rightFront, double rightBack) {
		setPowers(leftFront * mult, leftBack * mult, rightFront * mult, rightBack * mult);
	}

	private void setPowers(double leftFront, double leftBack, double rightFront, double rightBack) {
		this.leftFront.setPower(leftFront);
		this.leftBack.setPower(leftBack);
		this.rightFront.setPower(rightFront);
		this.rightBack.setPower(rightBack);
	}
}