package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Nathaniel on 11/10/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Chan TeleOp", group="TestTeleOp")

public class Chan_TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareK9bot2   robot           = new HardwareK9bot2();              // Use a K9'shardware
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    //   double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    //  final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo

    @Override
    public void runOpMode() {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            if (gamepad1.right_bumper) {

                robot.motorFR.setPower(-.5);
                robot.motorFL.setPower(.5);
                robot.motorBL.setPower(-.5);
                robot.motorBR.setPower(.5);
            }

            else if (gamepad1.left_bumper) {
                robot.motorFR.setPower(.5);
                robot.motorFL.setPower(-.5);
                robot.motorBL.setPower(.5);
                robot.motorBR.setPower(-.5);
            }


            {
                float yValue = gamepad1.left_stick_y;

                float xValue = gamepad1.right_stick_y;

                ///the usadjieiwebiqeuhowfeodfqwiwewenqwnrnower jkdfnasikjfmxbnc liawekjsmhdbf iukjqd


                xValue = Range.clip(xValue, -1, 1);
                yValue = Range.clip(yValue, -1, 1);


                robot.motorFL.setPower(-yValue*.5);
                robot.motorBL.setPower(-yValue*.5);
                robot.motorFR.setPower(-xValue*.5);
                robot.motorBR.setPower(-xValue*.5);
            }

            // Use gamepad Y & A raise and lower the arm
            // if (gamepad1.right_bumper)
            //       armPosition += ARM_SPEED;
            //     else if (gamepad1.left_bumper)
            //           armPosition -= ARM_SPEED;

            // Use gamepad X & B to open and close the claw
            if (gamepad1.x)
                armPosition += ARM_SPEED;
            else if (gamepad1.b)
                armPosition -= ARM_SPEED;

            // Move both servos to new position.
            armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
            robot.arm.setPosition(armPosition);
            // clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            // robot.claw.setPosition(clawPosition);

            // Send telemetry message to signify robot running;
            telemetry.addData("arm",   "%.2f", armPosition);
            //  telemetry.addData("claw",  "%.2f", clawPosition);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}

