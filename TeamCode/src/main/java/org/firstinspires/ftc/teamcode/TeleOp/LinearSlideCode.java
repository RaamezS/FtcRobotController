package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LinearSlideCode", group="Robot")
//@Disabled
public class LinearSlideCode extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor slide = null; //the slide motor

    final double SLIDE_EXTEND = -1.0;
    final double SLIDE_OFF =  0.0;
    final double SLIDE_BACK =  0.5;

    @Override
    public void runOpMode() {

        slide = hardwareMap.get(DcMotor.class, "Slide");
        slide.setPower(SLIDE_OFF);

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        /* Run while holding dpad.up or down*/
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                telemetry.addLine("Robot extending");
                telemetry.update();
                slide.setPower(SLIDE_EXTEND);
            }
            else if (gamepad1.dpad_down) {
                telemetry.addLine("Robot retracting");
                telemetry.update();
                slide.setPower(SLIDE_BACK);
            }
            else {
                slide.setPower(SLIDE_OFF);
            }
        }
    }
}
