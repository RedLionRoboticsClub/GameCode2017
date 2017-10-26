package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="Tom Autonomous", group ="TestsOpMode")


public class Tom_Autonomous extends LinearOpMode
{

    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorFL;
    DcMotor motorFR;
//    private static final int MOTOR_TICKS = 1220;
    // in cm
//    private static final double WHEEL_CIRCUMFERENCE = 32.5;
//    private static final double ROBOT_WIDTH = 37.5;
//    private static final double ROBOT_LENGTH = 37.5;

    @Override
    public void runOpMode() throws InterruptedException {

        motorBL  = this.hardwareMap.dcMotor.get("motorBL");
        motorBR = this.hardwareMap.dcMotor.get("motorBR");
        motorFL = this.hardwareMap.dcMotor.get("motorFL");
        motorFR = this.hardwareMap.dcMotor.get("motorFR");

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        // 37 cm from center of front wheels to 91 cm minus radius (half of width, 19 cm)
        moveMotors(.5,.5,.5,.5,2000,2000,2000,2000);
        reverseMotors(.5,.5,.5,.5,2000,2000,2000,2000);
        rightMotors(.5,.5,.5,.5,2000,2000,2000,2000);
        leftMotors(.5,.5,.5,.5,2000,2000,2000,2000);

        //write thing here to make
        //moveMotors(1/4, 0, (int) Math.round(ROBOT_WIDTH*Math.PI/4), 0);

    }

    /**
     * Converts a distance of centimeters into ticks for the encoders of the motors
     *
     * @param cm the distance in centimeters
     * @return a rounded value of the equivalent amount of distance in ticks
     */
//    int distance (double cm)


    /**
     * tells the motors to move at a certain speed (power) and to a specific distance in cm
     *
     * @param powerBL the amount of power that the left motor should use [-1.0, 1.0]
     * @param powerBR the amount of power that the right motor should use [-1.0, 1.0]
     * @param posBL the amount in centimeters that the left wheel should turn
     * @param posBR the amount in centimeters that the right wheel should turn
     * @throws InterruptedException
     */
    void moveMotors (double powerBL, double powerBR, double powerFL, double powerFR,
                     int posBL, int posBR, int posFL, int posFR) throws InterruptedException
    {
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorBL.setTargetPosition(posBL);
        motorBR.setTargetPosition(posBR);
        motorFR.setTargetPosition(posFR);
        motorFL.setTargetPosition(posFL);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
        motorFR.setPower(powerFR);
        motorFL.setPower(powerFL);
        // BUSYWAIT... Don't show this to our CS professors :S
        while (motorFL.getCurrentPosition() < posFL || motorFR.getCurrentPosition() < posFR || motorBL.getCurrentPosition() < posBL || motorBR.getCurrentPosition() < posBR)
        {
            Thread.sleep(20);
        }
        /**      motorBR.setPower(0.0);
         motorBL.setPower(0.0);
         motorFL.setPower(0);
         motorFR.setPower(0);
         motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);             **/
        // target has reached destination
//        motorLeft.setPower(0.0);
//        motorRight.setPower(0.0);
//        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    void rightMotors(double powerBL, double powerBR, double powerFL, double powerFR,
                     int posBL, int posBR, int posFL, int posFR) throws InterruptedException{
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);

        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorBL.setTargetPosition(posBL);
        motorBR.setTargetPosition(posBR);
        motorFR.setTargetPosition(posFR);
        motorFL.setTargetPosition(posFL);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
        motorFR.setPower(powerFR);
        motorFL.setPower(powerFL);
        // BUSYWAIT... Don't show this to our CS professors :S
        while (motorFL.getCurrentPosition() < posFL || motorFR.getCurrentPosition() < posFR || motorBL.getCurrentPosition() < posBL || motorBR.getCurrentPosition() < posBR)
        {
            Thread.sleep(20);
        }
    }

    void leftMotors(double powerBL, double powerBR, double powerFL, double powerFR,
                    int posBL, int posBR, int posFL, int posFR) throws InterruptedException{
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorBL.setTargetPosition(posBL);
        motorBR.setTargetPosition(posBR);
        motorFR.setTargetPosition(posFR);
        motorFL.setTargetPosition(posFL);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
        motorFR.setPower(powerFR);
        motorFL.setPower(powerFL);
        // BUSYWAIT... Don't show this to our CS professors :S
        while (motorFL.getCurrentPosition() < posFL || motorFR.getCurrentPosition() < posFR || motorBL.getCurrentPosition() < posBL || motorBR.getCurrentPosition() < posBR)
        {
            Thread.sleep(20);
        }
    }


    void reverseMotors(double powerBL, double powerBR, double powerFL, double powerFR,
                       int posBL, int posBR, int posFL, int posFR) throws InterruptedException{
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);

        motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorBL.setTargetPosition(posBL);
        motorBR.setTargetPosition(posBR);
        motorFR.setTargetPosition(posFR);
        motorFL.setTargetPosition(posFL);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
        motorFR.setPower(powerFR);
        motorFL.setPower(powerFL);
        // BUSYWAIT... Don't show this to our CS professors :S
        while (motorFL.getCurrentPosition() < posFL || motorFR.getCurrentPosition() < posFR || motorBL.getCurrentPosition() < posBL || motorBR.getCurrentPosition() < posBR)
        {
            Thread.sleep(20);
        }
    }
    // assumes power is in a step of a tenth...
    void stopMotors () throws InterruptedException
    {
        double initpowerL = motorBL.getPower() * 10;
        double initpowerR = motorBR.getPower() * 10;
        for (int i = 0; i < Math.max(Math.abs(initpowerL), Math.abs(initpowerR)); i++)
        {
            int signLeft = (motorBL.getPower() < 0)? -1 : 1;
            int signRight = (motorBR.getPower() < 0)? -1 : 1;
            motorBL.setPower(signLeft * Math.abs(motorBL.getPower()) - 0.1);
            motorBR.setPower(signRight * Math.abs(motorBR.getPower()) - 0.1);
            Thread.sleep(50);
        }
    }

}