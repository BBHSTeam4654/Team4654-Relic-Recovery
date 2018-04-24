package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//this serves as a base make a blue and red forward and back if you wish
//I seperated my P code to make it easy to do other stuff if you want
public class PoPMode extends LinearBaseOpMode {

    final int MARGINOFERROR = 42; //written in  encoder ticks
    final float P = .1f; //I have no idea whether this is a good value
    final int DISTANCEMARGIN = 42; //also encoder ticks not necessarily same is MOE
    final float POWER = 1.0f;


    static int leftDistance = 0;
    static int rightDistance = 0;

    public static boolean forwards = true;

    public static int distance = 0; //rotations * 420 * 2 * pi * wheel radius
    //wheel radius is 2in or 50.8mm
    //you could also integrate this one's doP into another opMode
    @Override
    public void runOpMode() throws InterruptedException {
    super.runOpMode();

        while (!this.isStopRequested()) {
            doP(true);

        }

    }

    public void doP(boolean forwards) {
        //left back right back
        int leftOld, rightOld;

        leftOld = leftBack.getCurrentPosition();
        rightOld = rightBack.getCurrentPosition();
        //Neverest 60 is 420 ppr

        if (forwards) {
            leftDistance += leftBack.getCurrentPosition() - leftOld;
            rightDistance += rightBack.getCurrentPosition() - rightOld;

            leftOld = leftBack.getCurrentPosition();
            rightOld = rightBack.getCurrentPosition();

            if (leftDistance - rightDistance < MARGINOFERROR) {
                setPowers(POWER + P * (leftDistance - rightDistance), POWER + P * (leftDistance - rightDistance),
                        POWER - P * (leftDistance - rightDistance), POWER - P * (leftDistance - rightDistance));
            } else if (rightDistance - leftDistance < MARGINOFERROR) {
                setPowers(POWER - P * (leftDistance - rightDistance), POWER - P * (leftDistance - rightDistance),
                        POWER + P * (leftDistance - rightDistance), POWER + P * (leftDistance - rightDistance));
            } else {
                setPowers(POWER, POWER, POWER, POWER);
            }


        } else {

            leftDistance += leftBack.getCurrentPosition() - leftOld;
            rightDistance += rightBack.getCurrentPosition() - rightOld;

            leftOld = leftBack.getCurrentPosition();
            rightOld = rightBack.getCurrentPosition();

            if (leftDistance - rightDistance < MARGINOFERROR) {
                setPowers(-POWER - P * (leftDistance - rightDistance), -POWER - P * (leftDistance - rightDistance),
                        -POWER + P * (leftDistance - rightDistance), -POWER + P * (leftDistance - rightDistance));
            } else if (rightDistance - leftDistance < MARGINOFERROR) {
                setPowers(-POWER + P * (leftDistance - rightDistance), -POWER + P * (leftDistance - rightDistance),
                        -POWER - P * (leftDistance - rightDistance), -POWER - P * (leftDistance - rightDistance));
            } else {
                setPowers(-POWER, -POWER, -POWER, -POWER);
            }
        }


        if(Math.abs((leftDistance+rightDistance)/2 - distance) < DISTANCEMARGIN ){
            //do some other stuff
            //like stopping if we get close
            }

    }

    protected void setPowers(double leftFront, double leftBack, double rightFront, double rightBack) {
        this.leftFront.setPower(leftFront);
        this.leftBack.setPower(leftBack);
        this.rightFront.setPower(rightFront);
        this.rightBack.setPower(rightBack);
    }
}
