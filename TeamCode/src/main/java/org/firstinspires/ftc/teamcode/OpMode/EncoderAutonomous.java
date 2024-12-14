package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name = "EncoderAutonomous", group = "Autonomous")
public class EncoderAutonomous extends LinearOpMode {
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, linearSlide;
    private CRServo intake;
    private DcMotorEx armMotor;
    private Servo wrist;
    double ticksPerInch;
    double armTicksPerDegree;
    double slideTicksPerInch;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(CRServo.class, "intake");
        armMotor = hardwareMap.get(DcMotorEx.class, "left_arm");
        linearSlide = hardwareMap.get(DcMotor.class, "Slide"); //TODO: change name
        wrist = hardwareMap.get(Servo.class, "wrist");

        //Variable for ticks per inch
        ticksPerInch = 45.2937013447;
        armTicksPerDegree = 14.66972 * 5;
        slideTicksPerInch =

        //reverse the motor to be backwards because of orientation
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        ResetEncoders();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        telemetry.addData("LeftFrontDrivePosition",leftFrontDrive.getCurrentPosition());
        //TODO: Put movement code here
        wrist.setPosition(0.18);
        Move(15, 0.3); //for moving backwards/other directions do negative inches
        Strafe(34,0.3);
        Turn(35,0.3); //for rotating to the right do negative degrees
        Arm(95,0.5);
        Move(13, 0.3);
        Slide(1600,-0.9); //negative power is for extending the slide and positive power is for retracting
        Arm(-5,0.5);
        Intake(2000, 0.5);
        Arm(10,0.5);
        Slide(1600,0.9);
        Turn(-11.5,0.3);
        Move(-17,0.3);
        Strafe(-35,0.3);
        Arm(-90,0.5);
        Move(1.5,0.3);
        Slide(1600,-0.3);
        Intake(3000,-1.0);
        Arm(90,0.5);
        Slide(1600,0.3);

        waitUntilMotorsStop();
    }
    //Move function:
    void Move(double inches, double power) {
        ResetEncoders();
        int ticks = (int)(inches * ticksPerInch);

        leftFrontDrive.setTargetPosition(ticks);
        leftBackDrive.setTargetPosition(ticks);
        rightFrontDrive.setTargetPosition(ticks);
        rightBackDrive.setTargetPosition(ticks);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ((DcMotorEx)rightBackDrive).setTargetPositionTolerance(10);

        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);

        while (opModeIsActive() && (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy())) {
            telemetry.addData("LeftFront Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFront Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBack Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBack Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
            sleep(10); // Optional sleep to prevent CPU overload
        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //Strafing function:
    void Strafe(double inches, double power) {
        ResetEncoders();
        int ticks = (int)(inches * ticksPerInch);
        //the following code is for strafing left,
        // do negative ticks for moving back
        // and when you call this function to strafe right set inches to negative
        leftFrontDrive.setTargetPosition(-ticks);
        leftBackDrive.setTargetPosition(ticks);
        rightFrontDrive.setTargetPosition(ticks);
        rightBackDrive.setTargetPosition(-ticks);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);

        while (opModeIsActive() && (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy())) {
            telemetry.addData("LeftFront Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFront Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBack Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBack Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
            sleep(10); // Optional sleep to prevent CPU overload
        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //Turn function:
    void Turn(double inches, double power) {
        ResetEncoders();
        int ticks = (int)(inches * ticksPerInch);
        //the following code is for turning left,
        // do negative ticks for left front and back in order to turn right
        // and when you call this function to turn right set inches to negative
        leftFrontDrive.setTargetPosition(ticks);
        leftBackDrive.setTargetPosition(ticks);
        rightFrontDrive.setTargetPosition(-ticks);
        rightBackDrive.setTargetPosition(-ticks);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        leftFrontDrive.setPower(power);

        while (opModeIsActive() && (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy())) {
            telemetry.addData("LeftFront Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFront Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBack Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBack Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
            sleep(10); // Optional sleep to prevent CPU overload
        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //Arm motor function:
    void Arm(double degrees, double power) {
        ResetEncoders();
        int ticks = (int)(degrees * armTicksPerDegree);
        //the following code is for moving the arm up,
        // do negative ticks in order to move the arm down
        armMotor.setTargetPosition(ticks);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(power);

        while (opModeIsActive() && (armMotor.isBusy())) {
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.update();
            sleep(10); // Optional sleep to prevent CPU overload
        }

        armMotor.setPower(0);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void Slide(int inches, double power) {
        ResetEncoders();
        int ticks = (int)(inches * slideTicksPerInch);
        //the following code is for moving the arm up,
        // do negative ticks in order to move the arm down
        linearSlide.setTargetPosition(ticks);

        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearSlide.setPower(power);

        while (opModeIsActive() && (linearSlide.isBusy())) {
            telemetry.addData("Slide Position", linearSlide.getCurrentPosition());
            telemetry.update();
            sleep(10); // Optional sleep to prevent CPU overload
        }

        linearSlide.setPower(0);

        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void ResetMode() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void Intake(int duration, double power) {
        intake.setPower(power);
        //sleep for duration so not instant
        sleep(duration);
        //stop the motors
        intake.setPower(0);
    }

    private void waitUntilMotorsStop() {
        while (opModeIsActive() && (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy())) {
            sleep(10); // Sleep to prevent overloading the CPU
        }
        ResetEncoders();
    }
    void ResetEncoders() {
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}