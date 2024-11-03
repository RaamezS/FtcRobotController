package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



@Autonomous(name = "Park and Unpark", group = "Autonomous")
public class ParkAndUnparkOpMode extends LinearOpMode {

    double power = 0.5;

    private DcMotor leftFront, rightFront, leftRear, rightRear;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // Set all motors to brake when power is zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // Unpark: Move forward to unpark
        moveForward(0.5, 1000); // Adjust power and time as needed
        stopMotors();

        sleep(1000); // Pause before parking

        // Park: Move backward to park
        moveBackward(0.5, 1000); // Adjust power and time as needed
        stopMotors();
    }

    // Helper methods for movement
    private void moveForward(double power, int duration) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        sleep(duration);
    }

    private void moveBackward(double power, int duration) {
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftRear.setPower(-power);
        rightRear.setPower(-power);
        sleep(duration);
    }

    private void stopMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}

