package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoSampleBasket", group = "Autonomous")
public class AutoSampleBasket extends LinearOpMode {

    double power = 0.5;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, linearSlide;
    private CRServo intake;
    private DcMotorEx armMotor;
    private Servo wrist;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(CRServo.class, "intake");
        armMotor = hardwareMap.get(DcMotorEx.class, "left_arm");
        linearSlide = hardwareMap.get(DcMotorEx.class, "Slide");
        wrist = hardwareMap.get(Servo.class, "wrist");

        //reverse the motor to be backwards because of orientation
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //wait until you hit start to go
        waitForStart();

        //TODO: put the code for the movement here
        wristRotation(wristOperations.WRIST_OUT);
        move(movement.FORWARD, 0.2, 1300);
        move(movement.LEFT, 0.33, 2500);
        TurningDrivetrain(turningOperations.LEFT,0.2,4300);
        move(movement.BACKWARDS, 0.33, 70);
        ArmRotation(armOperations.ARM_FORWARD, 1, 1900);
        move(movement.FORWARD, 0.33, 950);
        slideExtension(slideOperations.SLIDE_FORWARD, 0.9, 1600);
        IntakeRotation(intakeOperations.INTAKE_OUT, 0.5, 1500);
        stopIntake();
        /*slideExtension(slideOperations.SLIDE_BACKWARDS, 0.9, 1600);
        ArmRotation(armOperations.ARM_FORWARD,1,250);
        move(movement.BACKWARDS, 0.33, 900);
        TurningDrivetrain(turningOperations.RIGHT,0.2,400);
        move(movement.RIGHT, 0.33, 4000);
        move(movement.BACKWARDS,0.33,400);*/
        // example: IntakeRotation(intakeOperations.INTAKE_IN, 0.5f, 1);
    }

    enum movement {
        FORWARD,
        BACKWARDS,
        LEFT,
        RIGHT
    }
    enum turningOperations {
        LEFT,
        RIGHT
    }
    enum intakeOperations {
        INTAKE_IN,
        INTAKE_OUT
    }

    enum armOperations {
        ARM_FORWARD,
        ARM_BACKWARDS
    }
    enum slideOperations {
        SLIDE_FORWARD,
        SLIDE_BACKWARDS
    }
    enum wristOperations {
        WRIST_OUT,
        WRIST_IN
    }
    private void move(movement movement, double power, int duration) {
        if(movement == AutoSampleBasket.movement.FORWARD)  {
            //set all motors to positive power to move forward
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);

        } else if(movement == AutoSampleBasket.movement.BACKWARDS) {
            //set all motors to negative power to move backwards
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);

        } else if(movement == AutoSampleBasket.movement.LEFT) {
            //set diagonal motors to negative to move left
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);

        } else if(movement == AutoSampleBasket.movement.RIGHT) {
            //set diagonal motors to positive to move right
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
        }
        //sleep for duration so not instant
        sleep(duration);
        //stop the motors
        stopMotors();
    }
    void TurningDrivetrain(turningOperations operation, double power, int duration) {
        if(operation == turningOperations.RIGHT) {
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
        }
        if(operation == turningOperations.LEFT) {
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
        }
        sleep(duration);
        stopMotors();
    }
    void wristRotation(wristOperations operation) {
        final double WRIST_FOLDED_IN   = 0.5;
        final double WRIST_FOLDED_OUT  = 0.18;
        if(operation == wristOperations.WRIST_IN) {
            wrist.setPosition(WRIST_FOLDED_IN);
        }
        if(operation == wristOperations.WRIST_OUT) {
            wrist.setPosition(WRIST_FOLDED_OUT);
        }
    }
    void slideExtension(slideOperations operation, double power, int duration) {
        if(operation == slideOperations.SLIDE_FORWARD) {
            linearSlide.setPower(power);
        }
        else if(operation == slideOperations.SLIDE_BACKWARDS) {
            linearSlide.setPower(-power);
        }
        sleep(duration);
        //stop it from moving
        stopSlide();
    }
    private void IntakeRotation(intakeOperations operation, double power, int duration) {
        if(operation == intakeOperations.INTAKE_IN) {
            //move the intake in
            intake.setPower(-power);
        } else if(operation == intakeOperations.INTAKE_OUT) {
            //move the intake out
            intake.setPower(power);
        }
        //sleep for duration so not instant
        sleep(duration);
        //stop the motors
        stopIntake();
    }
    private void ArmRotation(armOperations operation, double power, int duration) {
        //if it's operation is = to forwards, make it positive
        if(operation == armOperations.ARM_FORWARD) {
            armMotor.setPower(power);
        } else if(operation == armOperations.ARM_BACKWARDS) {
            //if = to backwards, back it negative
            armMotor.setPower(-power);
        }
        //sleep for duration so not instant
        sleep(duration);
        //stop the motors
        stopArm();
    }

    private void stopMotors() {
        //stop all motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void stopIntake() {
        //stop the intake
        intake.setPower(0);
    }
    private void stopArm() {
        //stop the arm
        armMotor.setPower(0);
    }
    private void stopSlide() {
        //stop the linear slide
        linearSlide.setPower(0);
    }
}