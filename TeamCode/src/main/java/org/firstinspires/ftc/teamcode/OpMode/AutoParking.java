package org.firstinspires.ftc.robotcontroller.external.samples;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



@Autonomous(name = "AutoParking", group = "Autonomous")
public class AutoParking extends LinearOpMode {

    double power = 0.5;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to brake when power is zero
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // Unpark: Move forward to unpark
        move(movement.FORWARD, 0.5, 100); // Adjust power and time as needed
        move(movement.RIGHT, 0.5, 2300);
        move(movement.BACKWARDS, 0.5, 50);


        sleep(1000); // Pause before parking

        // Park: Move backward to park
        /*moveBackward(0.5, 1000); // Adjust power and time as needed
        stopMotors();*/
    }

    enum movement {
        FORWARD,
        BACKWARDS,
        LEFT,
        RIGHT
    }

    private void move(movement movement, double power, int duration) {
        if(movement.equals(AutoParking.movement.FORWARD))  {
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);

        } else if(movement.equals(AutoParking.movement.BACKWARDS)) {
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);

        } else if(movement.equals(AutoParking.movement.LEFT)) {
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);

        } else if(movement.equals(AutoParking.movement.RIGHT)) {
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
        }
        sleep(duration);
        stopMotors();
    }

    // Helper methods for movement
    private void moveForward(double power, int duration) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(duration);
    }

    private void moveBackward(double power, int duration) {
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        sleep(duration);
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void moveRight(double power, int duration) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        sleep(duration);
    }
}
