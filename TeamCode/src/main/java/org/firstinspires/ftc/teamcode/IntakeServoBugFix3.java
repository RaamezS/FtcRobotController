package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Intake Servo Control", group="Linear Opmode")
public class IntakeServoBugFix3 extends LinearOpMode {

    private Servo intakeServo;
    private ElapsedTime timer = new ElapsedTime();

    // Define a flag to check the state of the servo
    private boolean isSpinningForward = false;
    private boolean isSpinningReverse = false;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        // Set the initial position of the servo
        intakeServo.setPosition(0.5); // Neutral position

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // Check if the "a" button is pressed for continuous spin forward
            if (gamepad1.a && !isSpinningForward) {
                intakeServo.setPosition(1.0); // Spin forward
                isSpinningForward = true;
                isSpinningReverse = false;
            }

            // Check if the "b" button is pressed for continuous spin in reverse
            else if (gamepad1.b && !isSpinningReverse) {
                intakeServo.setPosition(0.0); // Spin reverse
                isSpinningReverse = true;
                isSpinningForward = false;
            }

            // Check if the "x" button is pressed to stop the servo
            else if (gamepad1.x) {
                intakeServo.setPosition(0.5); // Stop in place (neutral)
                isSpinningForward = false;
                isSpinningReverse = false;
            }

            // Display telemetry data for debugging
            telemetry.addData("Servo Position", intakeServo.getPosition());
            telemetry.addData("Timer", timer.seconds());
            telemetry.update();
        }
    }
}
