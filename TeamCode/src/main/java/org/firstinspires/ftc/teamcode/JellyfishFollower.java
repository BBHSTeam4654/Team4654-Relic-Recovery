/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This 2016-2017 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="Jellyfish Follower", group ="Concept")
// @Disabled
public class JellyfishFollower extends LinearOpMode {

	private final ColorMode colorMode;

	public JellyfishFollower(ColorMode colorMode){
		this.colorMode = colorMode;
	}

    public JellyfishFollower() {
        this(ColorMode.RED);
    }

	public static final String TAG = "Jellyfish Finder";

	private OpenGLMatrix lastLocation = null;

	/**
	 * vuforia is the variable we will use to store our instance of the Vuforia
	 * localization engine.
	 */
	private ClosableVuforiaLocalizer vuforia;

	private BNO055IMU imu;
	private DcMotor[] motors = new DcMotor[4];
	private ColorSensor color;
	// private TouchSensor touch;
	private Servo colorArm, glyph;
//    private Servo touchArm;

	private enum ColorMode {
		RED,
		BLUE;
	}

	@Override
	public void runOpMode() {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

		// OR...  Do Not Activate the Camera Monitor View, to save power
		// VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		color = hardwareMap.colorSensor.get("colorSensor");
		colorArm = hardwareMap.servo.get("colorArm");
		// touch = hardwareMap.touchSensor.get("touchSensor");
		// touchArm = hardwareMap.servo.get("touchArm");
		glyph = hardwareMap.servo.get("glyphDrop");
		colorArm.setPosition(0.6);
//        touchArm.setPosition(0); // TODO: Find correct position
		glyph.setPosition(0.4);

		final String[] names = {"leftFront", "leftBack", "rightFront", "rightBack"};
		for (int i = 0; i < motors.length; i++) {
			motors[i] = hardwareMap.dcMotor.get(names[i]);
		}
		motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
		motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

		// Set up the parameters with which we will use our IMU. Note that integration
		// algorithm here just reports accelerations to the logcat log; it doesn't actually
		// provide positional information.
		BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
		parametersIMU.angleUnit		   = BNO055IMU.AngleUnit.DEGREES;
		parametersIMU.accelUnit		   = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
		parametersIMU.loggingEnabled	  = true;
		parametersIMU.loggingTag		  = "IMU";
		parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

		// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
		// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
		// and named "imu".
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parametersIMU);

		parameters.vuforiaLicenseKey = "AV7cAYn/////AAAAGXDR1Nv900lOoewPO1Nq3ypDBIfk+d8X+UJOgVQZn5ZvQIY5Y4yGL6DVf24bEoMOVLCq5sZXPs9937r2zpeSZQaaaJbxeWggveVuvccsVlBdR38brId6fIRi/ssxtkUpVppCaRDO1N6K7IVbAJWrhpv1rG2DqTcS51znxjEYDE34AN6sNkurIq/qs0tLfvI+lx5VYRKdqh5LwnVt2HnpdX836kSbAN/1wnupzlLSKHcVPF9zlmRjCXrHduW8ikVefKAPGNCEzaDj4D+X+YM9iaHj9H8qN23bbaT81Ze3g5WwrXsb6dsX1N3+FqeXbiEUB02lXsmGwtvCJI89xutgPzlDAHqerduaLS2WZbL3oVyS";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		//this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
		this.vuforia = new ClosableVuforiaLocalizer(parameters);

        /*
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen). You can make your own datasets
         * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager.
         */
		VuforiaTrackables fieldTargets = this.vuforia.loadTrackablesFromAsset("RelicRecovery");
		fieldTargets.get(0).setName("2");
		fieldTargets.get(1).setName("1");
		fieldTargets.get(2).setName("0");

//        VuforiaTrackable jellyfish = fieldTargets.get(0);
//        jellyfish.setName("Jellyfish");  // Stones
//
//        /* For convenience, gather together all the trackable objects in one easily-iterable collection */
//        List<VuforiaTrackable> allTrackables = new ArrayList<>();
//        allTrackables.add(jellyfish);

        /*
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
		float mmPerInch        = 25.4f;
		float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
		float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        /*
         * In order for localization to work, we need to tell the system where each target we
         * wish to use for navigation resides on the field, and we need to specify where on the robot
         * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * For the most part, you don't need to understand the details of the math of how transformation
         * matrices work inside (as fascinating as that is, truly). Just remember these key points:
         * <ol>
         *
         *     <li>You can put two transformations together to produce a third that combines the effect of
         *     both of them. If, for example, you have a rotation transform R and a translation transform T,
         *     then the combined transformation matrix RT which does the rotation first and then the translation
         *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
         *     <em>reverse</em> of the chronological order in which they applied.</li>
         *
         *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
         *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
         *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
         *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
         *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
         *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
         *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
         *
         *     <li>If you want to break open the black box of a transformation matrix to understand
         *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
         *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
         *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
         *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
         *
         * </ol>
         *
         * This example places the "stones" image on the perimeter wall to the Left
         *  of the Red Driver station wall.  Similar to the Red Beacon Location on the Res-Q
         *
         * This example places the "chips" image on the perimeter wall to the Right
         *  of the Blue Driver station.  Similar to the Blue Beacon Location on the Res-Q
         *
         * See the doc folder of this project for a description of the field Axis conventions.
         *
         * Initially the target is conceptually lying at the origin of the field's coordinate system
         * (the center of the field), facing up.
         *
         * In this configuration, the target's coordinate system aligns with that of the field.
         *
         * In a real situation we'd also account for the vertical (Z) offset of the target,
         * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
         *
         * To place the Stones Target on the Red Audience wall:
         * - First we rotate it 90 around the field's X axis to flip it upright
         * - Then we rotate it  90 around the field's Z access to face it away from the audience.
         * - Finally, we translate it back along the X axis towards the red audience wall.
         */
//        OpenGLMatrix jellyfishLocation = OpenGLMatrix
//                /* Then we translate the target off to the RED WALL. Our translation here
//                is a negative translation in X.*/
//                .translation(0, 609.6F, 0)
//                .multiplied(Orientation.getRotationMatrix(
//                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
//                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
//                        AngleUnit.DEGREES, 90, 0, 0));
//        jellyfish.setLocation(jellyfishLocation);
//        RobotLog.ii(TAG, "Red Target=%s", format(jellyfishLocation));

       /*
        * To place the Stones Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
//        OpenGLMatrix blueTargetLocationOnField = OpenGLMatrix
//                /* Then we translate the target off to the Blue Audience wall.
//                Our translation here is a positive translation in Y.*/
//                .translation(0, mmFTCFieldWidth/2, 0)
//                .multiplied(Orientation.getRotationMatrix(
//                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//        blueTarget.setLocation(blueTargetLocationOnField);
//        RobotLog.ii(TAG, "Blue Target=%s", format(blueTargetLocationOnField));

        /*
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
		OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
				.translation(0,0,0)
				.multiplied(Orientation.getRotationMatrix(
						AxesReference.EXTRINSIC, AxesOrder.XZY,
						AngleUnit.DEGREES, 90, -90, 0));
		RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        /*
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
		//((VuforiaTrackableDefaultListener) jellyfish.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
		//((VuforiaTrackableDefaultListener)blueTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /*
         * A brief tutorial: here's how all the math is going to work:
         *
         * C = phoneLocationOnRobot  maps   phone coords -> robot coords
         * P = tracker.getPose()     maps   image target coords -> phone coords
         * L = redTargetLocationOnField maps   image target coords -> field coords
         *
         * So
         *
         * C.inverted()              maps   robot coords -> phone coords
         * P.inverted()              maps   phone coords -> imageTarget coords
         *
         * Putting that all together,
         *
         * L x P.inverted() x C.inverted() maps robot coords to field coords.
         *
         * @see VuforiaTrackableDefaultListener#getRobotLocation()
         */

        /* Wait for the game to begin */
		telemetry.addData(">", "Press Play to start tracking");
		telemetry.update();
		waitForStart();

        /* Start tracking the data sets we care about. */
        fieldTargets.activate();

        double initialRot = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
        CryptoboxDetector crypto = null;
        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
        VectorF pos = null;
        Orientation rot = null;
        int column = -1;

        telemetry.log().add("State: VUFORIA");
        vuforia: while (opModeIsActive()) {
            for (VuforiaTrackable trackable : fieldTargets) {
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    column = Integer.parseInt(trackable.getName());
                    telemetry.log().add("Column " + column);
                }
				vuforia.close();
				break vuforia;
            }
        }
        sleep(5000);

        // COLOR
        telemetry.log().add("State: COLOR");
        colorArm.setPosition(.064);
        sleep(1000);
        int red = color.red();
        int blue = color.blue();
        if ((red > blue) == (colorMode == ColorMode.RED)) {
            telemetry.log().add("Red");
            setPowers(0.2);
        } else {
            telemetry.log().add("Blue");
            setPowers(-0.2);
        }
        sleep(777);
        setPowers(0, 0, 0, 0);
        colorArm.setPosition(0.7);

        // DOGECV
        crypto = new CryptoboxDetector();
        crypto.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        //crypto.downScaleFactor = 0.4; // IDK what this does
        crypto.detectionMode = CryptoboxDetector.CryptoboxDetectionMode.BLUE; // or red
        crypto.rotateMat = true;
        crypto.enable();
        sleep(5000);
        setPowers(0.2);

        while (opModeIsActive()) {
            telemetry.addData("State", "DOGECV");
            telemetry.addData("Left Position", crypto.getCryptoBoxLeftPosition());
            telemetry.addData("Center Position", crypto.getCryptoBoxCenterPosition());
            telemetry.addData("Right Position", crypto.getCryptoBoxRightPosition());
            telemetry.addData("Target Column", column);
            telemetry.addData("Width", crypto.getWidth());
            telemetry.update();

            if (Math.abs(crypto.getCryptoBoxPositions()[2 - column] - crypto.getWidth() / 2) < 5) {
                break;
            }
        }
        setPowers(0);
        sleep(3000);

        // ROTATE
        double lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
        double targetAngle = initialRot;
        while (targetAngle < -Math.PI) targetAngle += 2 * Math.PI;
        while (targetAngle > Math.PI) targetAngle -= 2 * Math.PI;

        // Calculates direction based on the fastest way to get to the target angle
        double direction = ((targetAngle - lastAngle > 0) ^ (Math.abs(targetAngle - lastAngle) > Math.PI)) ? 1.0 : -1.0;
        setPowers(0.2 * direction);
        while (opModeIsActive()) {
            double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;

            telemetry.addData("State", "ROTATE");
            telemetry.addData("Current Angle", angle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Angle Difference", angle - targetAngle);
            telemetry.addData("Direction", direction);
            telemetry.addData("Last Angle", lastAngle);
            telemetry.update();

            if (angle * direction >= targetAngle * direction && lastAngle * direction < targetAngle * direction) {
                setPowers(0);
                break;
            } else {
                setPowers(Math.min(0.2, 0.075 + direction * (targetAngle - angle)) * direction);
            }

            lastAngle = angle;
        }

        sleep(1000);
        // touchArm.setPosition(0.6); // TODO: Find correct position
        setMecanumPowers(-Math.PI / 2, 0.15);

        long start = System.currentTimeMillis();
//        while (opModeIsActive() && !touch.isPressed() && System.currentTimeMillis() - start < 3000);
		while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
			telemetry.addData("Time", System.currentTimeMillis() - start);
			sleep(1);
		}
        setPowers(0);
        glpyh.setPosition(0);
        // touchArm.setPosition(0); // TODO: Find correct position
        sleep(2000);
        setMecanumPowers(Math.PI / 2, 0.2);
        sleep(500);

        telemetry.log().add("DONE");
        fieldTargets.deactivate();
        crypto.disable();
        sleep(10000);

//                    double lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
//                    double targetAngle = lastAngle - rot.thirdAngle;
//                    while (targetAngle < -Math.PI) targetAngle += 2 * Math.PI;
//                    while (targetAngle > Math.PI) targetAngle -= 2 * Math.PI;
//
//                    // Calculates direction based on the fastest way to get to the target angle
//                    double direction = ((targetAngle - lastAngle > 0) ^ (Math.abs(targetAngle - lastAngle) > Math.PI)) ? 1.0 : -1.0;
//                    setPowers(0.2 * direction);
//
//                    while (opModeIsActive()) {
//                        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
//
//                        telemetry.addData("State", "ROTATE");
//                        telemetry.addData("Current Angle", angle);
//                        telemetry.addData("Target Angle", targetAngle);
//                        telemetry.addData("Angle Difference", angle - targetAngle);
//                        telemetry.addData("Direction", direction);
//                        telemetry.addData("Last Angle", lastAngle);
//                        telemetry.update();
//
//                        if (angle * direction >= targetAngle * direction && lastAngle * direction < targetAngle * direction) {
//                            setPowers(0);
//                            state = State.STOP;
//                            break;
//                        } else {
//                            setPowers(Math.min(0.2, 0.075 + direction * (targetAngle - angle)) * direction);
//                        }
//
//                        lastAngle = angle;
//                    }
	}

	private void setPowers(double power) {
		setPowers(power, power, power, power);
	}

	private void setPowers(double leftFront, double leftBack, double rightFront, double rightBack) {
		motors[0].setPower(leftFront);
		motors[1].setPower(leftBack);
		motors[2].setPower(rightFront);
		motors[3].setPower(rightBack);
	}

	private void setMecanumPowers(double angle, double power) {
		double sin = Math.sin(angle - Math.PI / 4);
		double cos = Math.cos(angle - Math.PI / 4);

		setPowers(power * sin, power * cos, power * cos, power * sin);
	}

	/**
	 * A simple utility that extracts positioning information from a transformation matrix
	 * and formats it in a form palatable to a human being.
	 */
	private String format(OpenGLMatrix transformationMatrix) {
		return transformationMatrix.formatAsTransform();
	}
}
