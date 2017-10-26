package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Nathaniel on 11/10/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Bryce TeleOp", group="TestTeleOp")

public class Bryce_TeleOp extends OpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBR;
    DcMotor motorBL;

    @Override
    public void init() {

        motorBL = this.hardwareMap.dcMotor.get("motorBL");
        motorBR = this.hardwareMap.dcMotor.get("motorBR");
        motorFL = this.hardwareMap.dcMotor.get("motorFL");
        motorFR = this.hardwareMap.dcMotor.get("motorFR");

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {




            if (gamepad1.right_bumper) {

                    motorFR.setPower(-1);
                    motorFL.setPower(1);
                    motorBL.setPower(-1);
                    motorBR.setPower(1);
                }

                else if (gamepad1.left_bumper) {
                    motorFR.setPower(1);
                    motorFL.setPower(-1);
                    motorBL.setPower(1);
                    motorBR.setPower(-1);
                }
                else if(gamepad1.x) {
                    motorFR.setPower(1);
            }

                else if(gamepad1.b) {
                     motorFL.setPower(1);
            }

                else if(gamepad1.y) {
                    motorBR.setPower(1);
            }

                else if(gamepad1.a) {
                    motorBL.setPower(1);
            }

                    else

                {
                    float yValue = gamepad1.left_stick_y;

                    float xValue = gamepad1.right_stick_y;

        ///the usadjieiwebiqeuhowfeodfqwiwewenqwnrnower jkdfnasikjfmxbnc liawekjsmhdbf iukjqd


                    xValue = Range.clip(xValue, -1, 1);
                    yValue = Range.clip(yValue, -1, 1);


                    motorFL.setPower(-yValue);
                    motorBL.setPower(-yValue);
                    motorFR.setPower(-xValue);
                    motorBR.setPower(-xValue);
                }

            }
        }