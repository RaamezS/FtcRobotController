package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "EncoderAutoNew", group = "Autonomous")
public class EncoderAutoNew extends LinearOpMode {
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, linearSlide;
    //private CRServo intake;
    private DcMotorEx armMotor;
    private Servo wrist, claw;
    double ticksPerInch;
    double armTicksPerDegree;
    double slideTicksPerInch;
    private ElapsedTime runtime = new ElapsedTime();
    double CLAW_OPEN = 0;
    double CLAW_CLOSE = 0.3;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        //intake = hardwareMap.get(CRServo.class, "intake");
        armMotor = hardwareMap.get(DcMotorEx.class, "left_arm");
        linearSlide = hardwareMap.get(DcMotor.class, "Slide"); //TODO: change name
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        //Variable for ticks per inch
        ticksPerInch = 45.2937013447;
        armTicksPerDegree = 14.66972 * 5;
        slideTicksPerInch = 751.8 / 4.72256;

        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse the motor to be backwards because of orientation
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setDirection(DcMotor.Direction.REVERSE);

        ResetEncoders();

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        telemetry.addData("LeftFrontDrivePosition",leftFrontDrive.getCurrentPosition());
        //TODO: Put movement code here
        //wrist.setPosition(0.18);
        claw.setPosition(CLAW_CLOSE);
        sleep(500);
        //Step 1: Move robot diagonally and raise arm
        //ArmAndDiagonal(90, 1.0, 100, 20,0.45);
        Move(10,0.4);
        ArmAndStrafe(90,1.0,23,0.3);

        //Step 2: Turn robot to align with basket and extend slide
        //Slide(25,1.0,2000);
        TurnAndSlide(26,1.0,2000,35,0.3);
        Move(2.5,0.4);

        //Step 3: Put sample in high basket
        //Arm(-5,0.3);
        //Intake(1500,0.5);
        Arm(-6,0.3);
        claw.setPosition(CLAW_OPEN);
        sleep(500);
        Arm(10,0.7);

        //Step 4: Retract slide after slight arm lift, turn robot + lower arm
        Slide(-26,0.9,2500);
        ArmAndTurn(-92.5,1.0,-38,0.4);


        //Step 5: Extend slide and pick up second sample
        Move(12,0.6);
        Slide(10,0.9,2000);
        claw.setPosition(CLAW_CLOSE);
        sleep(500);
        Slide(-10,0.9,2000);
        ArmAndTurn(93,1.0,38,0.4);
        Strafe(12,0.8);
        Move(9,0.5);
        Slide(25,1.0,2000);
        Arm(-8,0.8);
        claw.setPosition(CLAW_OPEN);
        sleep(500);
        //ArmAndDiagonal(90,0.9,20,100,0.3);
        //option 2
        //ArmAndTurn(90,0.9,41,0.3);
        //Intake(3000,-1.0);


        //Step 6: Raise arm to 90 degree angle for deposit position and align with basket
        //ArmAndTurn(90,0.9,35,0.3);

        //Step 7: Adjust robot distance and put sample in high basket
        /*Slide(15,0.9,2000);
        Arm(-5,0.9);
        //Move(1.5,0.3);
        //Arm(-5,0.3);
        claw.setPosition(CLAW_OPEN);
        sleep(500);

        /*
        //Step 8 repeat steps 4-7


        //Step 4 repeat: Retract slide after slight arm lift, turn robot + lower arm
        Slide(-20,0.9,2500);
        Move(-1.5,0.3);
        ArmAndTurn(-90,0.9,30,0.3);

        //Step 5 repeat: Extend slide and pick up second sample
        Slide(20,0.9,2000);
        //Intake(3000,-1.0);
        Claw(CLAW_CLOSE);

        //Step 6 repeat: Raise arm to 90 degree angle for deposit position and align with basket
        ArmAndTurn(90,0.9,-30,0.3);

        //Step 7 repeat: Adjust robot distance and put sample in high basket
        Slide(20,0.9,2000);
        Move(1.5,0.3);
        Arm(-5,0.3);
        Claw(CLAW_OPEN);*/

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
    void TurnAndSlide(double slideInches, double slidePower, int timeStop, double driveInches, double drivePower) {
        ResetEncoders();
        runtime.reset();

        int ticks = (int)(driveInches * ticksPerInch);
        int slideTicks = (int)(slideInches * slideTicksPerInch);

        leftBackDrive.setTargetPosition(ticks);
        rightFrontDrive.setTargetPosition(-ticks);
        leftFrontDrive.setTargetPosition(ticks);
        rightBackDrive.setTargetPosition(-ticks);

        linearSlide.setTargetPosition(slideTicks);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setPower(drivePower);
        leftBackDrive.setPower(drivePower);
        rightBackDrive.setPower(drivePower);
        leftFrontDrive.setPower(drivePower);

        linearSlide.setPower(slidePower);

        while ((linearSlide.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || leftFrontDrive.isBusy() || rightBackDrive.isBusy())) {
            /*if(!rightFrontDrive.isBusy() || !leftBackDrive.isBusy() || !leftFrontDrive.isBusy() || !rightBackDrive.isBusy()) {
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftFrontDrive.setPower(0);
            }
            if(!linearSlide.isBusy() || runtime.milliseconds() > timeStop) {
                linearSlide.setPower(0);
            }*/
            telemetry.addData("right front 2: ", rightFrontDrive.getCurrentPosition());
            telemetry.addData("left back 2:", leftBackDrive.getCurrentPosition());
            telemetry.update();
            sleep(10); // Optional sleep to prevent CPU overload
        }

        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        linearSlide.setPower(0);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    void ArmAndTurn(double armDegrees, double armPower, double driveInches, double drivePower) {
        ResetEncoders();
        int ticks = (int)(driveInches * ticksPerInch);
        int degrees = (int)(armDegrees * armTicksPerDegree);

        leftBackDrive.setTargetPosition(ticks);
        rightFrontDrive.setTargetPosition(-ticks);
        leftFrontDrive.setTargetPosition(ticks);
        rightBackDrive.setTargetPosition(-ticks);

        armMotor.setTargetPosition(degrees);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setPower(drivePower);
        leftBackDrive.setPower(drivePower);
        rightBackDrive.setPower(drivePower);
        leftFrontDrive.setPower(drivePower);

        armMotor.setPower(armPower);

        while (rightFrontDrive.isBusy() || leftBackDrive.isBusy() || leftFrontDrive.isBusy() || rightBackDrive.isBusy() || armMotor.isBusy()) {
            /*if(!rightFrontDrive.isBusy() || !leftBackDrive.isBusy()|| !leftFrontDrive.isBusy() || !rightBackDrive.isBusy() ) {
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
            }
            if(!armMotor.isBusy()) {
                armMotor.setPower(0);
            }*/
            sleep(10); // Optional sleep to prevent CPU overload
        }
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        armMotor.setPower(0);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void ArmAndDiagonal(double armDegrees, double armPower, double leftBackInches, double rightFrontInches, double drivePower) {
        ResetEncoders();
        int leftBackTicks = (int)(leftBackInches * ticksPerInch);
        int rightFrontTicks = (int)(rightFrontInches * ticksPerInch);
        int degrees = (int)(armDegrees * armTicksPerDegree);

        leftBackDrive.setTargetPosition(leftBackTicks);
        rightFrontDrive.setTargetPosition(rightFrontTicks);

        armMotor.setTargetPosition(degrees);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setPower(drivePower);
        leftBackDrive.setPower(drivePower);

        armMotor.setPower(armPower);

        telemetry.addData("right front before: ", rightFrontDrive.isBusy());
        telemetry.addData("left back before:", leftBackDrive.isBusy());

        while (rightFrontDrive.isBusy() || leftBackDrive.isBusy() || armMotor.isBusy()) {
            telemetry.addData("right front: ", rightFrontDrive.isBusy());
            telemetry.addData("left back:", leftBackDrive.isBusy());
            telemetry.update();
            sleep(10); // Optional sleep to prevent CPU overload
        }
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        armMotor.setPower(0);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void ArmAndStrafe(double armDegrees, double armPower, double driveInches, double drivePower) {
        ResetEncoders();
        int ticks = (int)(driveInches * ticksPerInch);
        int degrees = (int)(armDegrees * armTicksPerDegree);

        leftFrontDrive.setTargetPosition(-ticks);
        leftBackDrive.setTargetPosition(ticks);
        rightFrontDrive.setTargetPosition(ticks);
        rightBackDrive.setTargetPosition(-ticks);

        armMotor.setTargetPosition(degrees);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setPower(drivePower);
        leftBackDrive.setPower(drivePower);
        rightBackDrive.setPower(drivePower);
        leftFrontDrive.setPower(drivePower);

        armMotor.setPower(armPower);

        while (opModeIsActive() && (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy())) {
            telemetry.addData("LeftFront Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFront Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBack Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBack Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
            sleep(10); // Optional sleep to prevent CPU overload
        }
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        armMotor.setPower(0);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void Slide(int inches, double power, double timeStop) {
        ResetEncoders();
        runtime.reset();
        int ticks = (int)(inches * slideTicksPerInch);
        //the following code is for moving the arm up,
        // do negative ticks in order to move the arm down
        linearSlide.setTargetPosition(ticks);

        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearSlide.setPower(power);

        while (opModeIsActive() && (linearSlide.isBusy()) && runtime.milliseconds() < timeStop) {
            telemetry.addData("Slide Position", linearSlide.getCurrentPosition());
            telemetry.update();
            sleep(10); // Optional sleep to prevent CPU overload
        }

        linearSlide.setPower(0);

        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void SlideTime(int duration, double power) {
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide.setPower(power);
        sleep(duration);
        linearSlide.setPower(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void ResetMode() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*private void Intake(int duration, double power) {
        intake.setPower(power);
        //sleep for duration so not instant
        sleep(duration);
        //stop the motors
        intake.setPower(0);
    }*/

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
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}