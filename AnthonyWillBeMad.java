package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="AnthonyWillBeMad", group="Test")

public class AnthonyWillBeMad extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareK9bot2 robot           = new HardwareK9bot2();              // Use a K9'shardware
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    //   double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    //  final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.05 ;                            // sets rate to move servo

    double armPosition2 = robot.ARM_HOME2;

    @Override
    public void runOpMode() {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello, My name is GLaDOS");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            if (gamepad1.right_trigger > 0) {

                robot.motorFR.setPower(-0.5);
                robot.motorFL.setPower(0.5);
                robot.motorBL.setPower(-0.5);
                robot.motorBR.setPower(0.5);
            }

            else if (gamepad1.left_trigger > 0) {
                robot.motorFR.setPower(0.5);
                robot.motorFL.setPower(-0.5);
                robot.motorBL.setPower(0.5);
                robot.motorBR.setPower(-0.5);
            }

            else {
                float yValue = gamepad1.left_stick_y;

                float xValue = gamepad1.right_stick_y;


                xValue = Range.clip(xValue, -1, 1);
                yValue = Range.clip(yValue, -1, 1);

                if (gamepad1.left_bumper) {
                    robot.motorFL.setPower(-yValue/2);
                    robot.motorBL.setPower(-yValue/2);
                    robot.motorFR.setPower(-xValue/2);
                    robot.motorBR.setPower(-xValue/2);
                }

                else if (gamepad1.right_bumper) {
                    robot.motorFL.setPower(-yValue/4);
                    robot.motorBL.setPower(-yValue/4);
                    robot.motorFR.setPower(-xValue/4);
                    robot.motorBR.setPower(-xValue/4);
                }

                else if (gamepad1.dpad_down)  {
                    robot.motorFL.setPower(-.25);
                    robot.motorBL.setPower(-.25);
                    robot.motorFR.setPower(-.25);
                    robot.motorBR.setPower(-.25);
                }

                else if (gamepad1.dpad_up)  {
                    robot.motorFL.setPower(.25);
                    robot.motorBL.setPower(.25);
                    robot.motorFR.setPower(.25);
                    robot.motorBR.setPower(.25);
                }

                else if (gamepad1.dpad_right)  {
                    robot.motorFR.setPower(-0.5);
                    robot.motorFL.setPower(0.5);
                    robot.motorBL.setPower(-0.5);
                    robot.motorBR.setPower(0.5);
                }

                else if (gamepad1.dpad_left)  {
                    robot.motorFR.setPower(0.5);
                    robot.motorFL.setPower(-0.5);
                    robot.motorBL.setPower(0.5);
                    robot.motorBR.setPower(-0.5);
                }

                else {
                    robot.motorFL.setPower(-yValue);
                    robot.motorBL.setPower(-yValue);
                    robot.motorFR.setPower(-xValue);
                    robot.motorBR.setPower(-xValue);
                }

            }
            if (gamepad2.left_bumper) {
                if (gamepad2.a) {
                    robot.motorArm.setPower(-0.5);
                } else if (gamepad2.b) {
                    robot.motorArm.setPower(0.5);
                } else {
                    robot.motorArm.setPower(0);
                }
            }

            else if (gamepad2.right_bumper) {
                if (gamepad2.a) {
                    robot.motorArm.setPower(-0.25);
                } else if (gamepad2.b) {
                    robot.motorArm.setPower(0.25);
                } else {
                    robot.motorArm.setPower(0);
                }
            }

            else {
                if (gamepad2.a) {
                    robot.motorArm.setPower(-1);
                } else if (gamepad2.b) {
                    robot.motorArm.setPower(1);
                } else {
                    robot.motorArm.setPower(0);
                }
            }




            // Use gamepad Y & A raise and lower the arm
            // if (gamepad1.right_bumper)
            // armPosition += ARM_SPEED;
            // else if (gamepad1.left_bumper)
            // armPosition -= ARM_SPEED;

            // Use gamepad X & Y to open and close the claw
            if (gamepad2.y) {
                armPosition += ARM_SPEED;
                armPosition2 -= ARM_SPEED;
            }
            else if (gamepad2.x) {
                armPosition -= ARM_SPEED;
                armPosition2 += ARM_SPEED;
            }

            // Move both servos to new position.
            armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
            armPosition2  = Range.clip(armPosition2, robot.ARM_MIN_RANGE2, robot.ARM_MAX_RANGE2);
            robot.arm.setPosition(armPosition);
            robot.arm2.setPosition(armPosition2);
            // clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            // robot.claw.setPosition(clawPosition);

            // Send telemetry message to signify robot running;
            telemetry.addData("arm",   "%.2f", armPosition);
            telemetry.addData("arm2",   "%.2f", armPosition2);
            // telemetry.addData("claw",  "%.2f", clawPosition);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);

        }

    }

}

}
