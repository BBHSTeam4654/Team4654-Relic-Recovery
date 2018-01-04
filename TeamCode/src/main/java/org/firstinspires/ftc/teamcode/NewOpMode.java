package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="20172211")
public class NewOpMode extends OpMode {

    ColorSensor colorSensor;
    DistanceSensor distanceSensor;
    CRServo claw;
    DcMotor leftFront, rightFront, leftBack, rightBack, armElevation, armLength;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        claw = hardwareMap.crservo.get("claw");
        armElevation = hardwareMap.dcMotor.get("armUp");
        armLength = hardwareMap.dcMotor.get("armOut");
    }

    @Override
    public void loop() {
        telemetry.addData("Color Sensor", colorSensor.red() + ", " + colorSensor.green() + ", " + colorSensor.blue());
        telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.CM) + "cm");

        float clawValue = gamepad2.left_stick_x;
        telemetry.addData("Claw Power", clawValue * 0.1);
        claw.setPower(clawValue * 0.1);

        float armUpPower = gamepad2.right_stick_y;
        armElevation.setPower(armUpPower);

        float armOutPower = gamepad2.left_stick_y;
        armLength.setPower(armOutPower * 0.5);
    }

}
