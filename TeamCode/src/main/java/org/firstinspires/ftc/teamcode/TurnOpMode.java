package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="TurnOnBot")
public class TurnOpMode extends OpMode {

	private CRServo claw;
	private Servo arm;
	private ColorSensor color;
	private DcMotor leftFront, rightFront, leftBack, rightBack;

	private long startTime;


	BNO055IMU imu;

	// State used for updating telemetry
	Orientation angles;
	Acceleration gravity;

	//Setup angles
	double initialAngle; //Our initial angle
	double angleTo; //The anglewe turn to
	double currentAngle, lastAngle; //Our current angle

	//margins of error because 89.9 is close enough to 90
	double turnMargin = 1; //5 degrees

	@Override
	public void init() {
		claw = hardwareMap.crservo.get("claw");
		arm = hardwareMap.servo.get("arm");
		color = hardwareMap.colorSensor.get("colorSensor");
		leftFront = hardwareMap.dcMotor.get("leftFront");
		leftBack = hardwareMap.dcMotor.get("leftBack");
		rightFront = hardwareMap.dcMotor.get("rightFront");
		rightBack = hardwareMap.dcMotor.get("rightBack");

		arm.setPosition(0.1);

		// provide positional information.
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
		parameters.loggingEnabled = true;
		parameters.loggingTag = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

		// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
		// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
		// and named "imu".
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);

		// Set up our telemetry dashboard

		// Wait until we're told to go

	}

	public void start() {
		imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
		initialAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
		currentAngle = initialAngle;
		angleTo = initialAngle + 90;
		if (angleTo > 180)
			angleTo -= 360;

		setPowers(.2, .2, .2, .2);
	}

	public void loop() {
		Orientation orient = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
		Position pos = imu.getPosition();

		lastAngle = currentAngle;
		currentAngle = orient.thirdAngle;

		telemetry.addData("First Angle: ", orient.firstAngle);
		telemetry.addData("Second Angle: ", orient.secondAngle);
		telemetry.addData("Third Angle: ", orient.thirdAngle);
		telemetry.addData("Last Angle: ", lastAngle);
		telemetry.addData("Initial: ", initialAngle);
		telemetry.addData("Difference: ", (currentAngle - initialAngle));
		telemetry.addData("Target: ", angleTo);
		telemetry.addData("Position X: ", pos.x);
		telemetry.addData("Position Y: ", pos.y);
		telemetry.addData("Position Z: ", pos.z);

		if (currentAngle >= angleTo && lastAngle < angleTo) {
			setPowers(0, 0, 0, 0);
		}
	}



	private void setPowers(double leftFront, double leftBack, double rightFront, double rightBack) {
		this.leftFront.setPower(leftFront);
		this.leftBack.setPower(leftBack);
		this.rightFront.setPower(rightFront);
		this.rightBack.setPower(rightBack);
	}
}
