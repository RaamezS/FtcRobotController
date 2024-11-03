package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class RangeOfArmCode extends LinearOpMode {

    public DcMotor armMotor; // Motor for the arm

    // Define the limits (adjust these values according to your robot's configuration)
    final int ARM_MIN_POSITION = 0; // Minimum position of the arm
    final int ARM_MAX_POSITION = 1000; // Maximum position of the arm (change as needed)

    @Override
    public void runOpMode() {

        armMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            int currentPosition = armMotor.getCurrentPosition();

            // Check if the shoulder button is pressed to move the arm up
            if (gamepad1.right_trigger != 0) {
                if (currentPosition < ARM_MAX_POSITION) {
                    armMotor.setPower(1.0); // Move the arm up
                } else {
                    armMotor.setPower(0.0); // Stop the arm if it reaches the max position
                }
            }
            // Check if the left shoulder button is pressed to move the arm down
            else if (gamepad1.left_trigger != 0) {
                if (currentPosition > ARM_MIN_POSITION) {
                    armMotor.setPower(-1.0); // Move the arm down
                } else {
                    armMotor.setPower(0.0); // Stop the arm if it reaches the min position
                }
            }
            else {
                armMotor.setPower(0.0); // Stop the arm if no button is pressed
            }

            // Optional: Provide feedback via telemetry
            telemetry.addData("Arm Position", currentPosition);
            telemetry.update();
        }
    }
}
