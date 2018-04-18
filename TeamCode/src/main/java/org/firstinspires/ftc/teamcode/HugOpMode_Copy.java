package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import java.util.Locale;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="1HugOpMode", group="Iterative Opmode")
public class HugOpMode_Copy extends BaseOpMode {
    
    enum State {
        DRIVE,
        MECANUM,
        TANK,
        MECANUM2
    }

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    private State state = State.DRIVE;

    private long lastTime;
    private long alsoLastTime;
    
    int temp = 0;
    
    @Override
    public void start() {
        lastTime = System.currentTimeMillis();
        alsoLastTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        long time = System.currentTimeMillis();
        long deltaTime = time - lastTime;
        lastTime = time;
        
        if(temp == 0 && time - alsoLastTime  > 1000){
            claw.setPosition(0);
            temp = 1;
            alsoLastTime = time;
        }
        else if(temp == 1 &&  time - alsoLastTime > 1000){
            claw.setPosition(1.0);
            temp = 0;
            alsoLastTime = time;
        }
        
        telemetry.addData("claw", time - alsoLastTime);
        
        if (gamepad1.dpad_up)
            state = State.DRIVE;
        else if (gamepad1.dpad_right)
            state = State.MECANUM;
        else if (gamepad1.dpad_down)
            state = State.TANK;
        else if (gamepad1.dpad_left)
            state = State.MECANUM2;
        
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

                setPowers(mult * power, sin, cos, cos, sin);
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
                setPowers(mult, left,left,right,right);
                break;
        }
    
        telemetry.addData("State", state);
        // telemetry.addData("Color", color.argb() & 0b11111111);
        telemetry.addData("Hug", gamepad2.right_stick_y);
    }
    
    private void setPowers(double mult, double frontLeft, double backLeft, double frontRight, double backRight) {
        setPowers(frontLeft * mult, backLeft * mult, frontRight * mult, backRight * mult);
    }
}
