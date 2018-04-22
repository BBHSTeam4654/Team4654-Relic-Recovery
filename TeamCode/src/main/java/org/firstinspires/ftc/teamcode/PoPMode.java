package org.firstinspires.ftc.teamcode;

public class PoPMode extends LinearBaseOpMode {

    final int MARGINOFERROR = 10;
    final int P = 1; //I have no idea whether this is a good value

    final float POWER = 1.0f;


    static int leftDistance = 0;
    static int rightDistance = 0;

    @Override
    public void runOpMode() {


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
                setPowers(POWER, POWER, POWER, POWER);
            }
        }


    }

    protected void setPowers(double leftFront, double leftBack, double rightFront, double rightBack) {
        this.leftFront.setPower(leftFront);
        this.leftBack.setPower(leftBack);
        this.rightFront.setPower(rightFront);
        this.rightBack.setPower(rightBack);
    }
}
