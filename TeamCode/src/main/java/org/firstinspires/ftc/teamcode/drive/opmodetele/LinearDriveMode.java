package org.firstinspires.ftc.teamcode.drive.opmodetele;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.robot.Robot;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    int direction = 1; //daca e true e in fata daca e false e in spate
    double servoPowSlides = 0.5;
    double servoPosGrippy = 0;
    double servoPosAngle = 0.5;
    boolean craneHold;
    int cranePosition = 0;

    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * Math.pow(100 * (abs(x) / 100), 2);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }
        //INIT CODE
        telemetry.addData(">", "Initialized");
        telemetry.update();
//        robot.crane.motorCraneLeft.setTargetPosition(cranePosition);
//        robot.crane.motorCraneRight.setTargetPosition(cranePosition);
//        robot.crane.motorCraneLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.crane.motorCraneRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //TELEOP CODE

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //GAMEPAD 2

            //EXTEND AND RETRACT SLIDES
            if (gamepad2.left_bumper) {
                servoPowSlides = -1;
                robot.crane.moveSlides(servoPowSlides);
            } else if (gamepad2.right_bumper) {
                servoPowSlides = 1;
                robot.crane.moveSlides(servoPowSlides);
            } else {
                servoPowSlides = 0;
                robot.crane.moveSlides(servoPowSlides);
            }

            //MOVE THE ENTIRE CRANE
//            if (gamepad2.left_trigger > 0.1) {
//                robot.crane.manualTarget = robot.crane.motorCraneRight.getCurrentPosition() - calculateThrottle(gamepad2.left_trigger * 5);
//                robot.crane.manualTarget--;
//                robot.crane.manualLevel(robot.crane.manualTarget);
//            }
//            if (gamepad2.right_trigger > 0.1) {
//                robot.crane.manualTarget = robot.crane.motorCraneRight.getCurrentPosition() + calculateThrottle(gamepad2.right_trigger * 5);
//                robot.crane.manualTarget++;
//                robot.crane.manualLevel(robot.crane.manualTarget);
//            }
//            if(gamepad2.left_trigger > 0.1){
//                craneHold = false;
//                robot.crane.motorCraneLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.crane.motorCraneRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.crane.motorCraneLeft.setPower(-0.7);
//                robot.crane.motorCraneRight.setPower(-0.7);
//            }
//            else if(gamepad2.right_trigger > 0.1){
//                craneHold = false;
//                robot.crane.motorCraneLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.crane.motorCraneRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.crane.motorCraneRight.setPower(0.7);
//                robot.crane.motorCraneLeft.setPower(0.7);
//            }
//            else{
//                if(!craneHold){
//                    craneHold = true;
//                    robot.crane.motorCraneRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    robot.crane.motorCraneLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    robot.crane.motorCraneLeft.setTargetPosition(0);
//                    robot.crane.motorCraneRight.setTargetPosition(0);
//                    robot.crane.motorCraneLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.crane.motorCraneRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                }
//            }

            if(gamepad2.left_trigger > 0.1){
                cranePosition ++;
            }
            else if(gamepad2.right_trigger > 0.1){
                cranePosition --;
            }
//            robot.crane.motorCraneLeft.setTargetPosition(cranePosition);
//            robot.crane.motorCraneRight.setTargetPosition(cranePosition);

//            OPEN AND CLOSE THE GRIPPER
            if (gamepad2.a) {
                robot.crane.setGripper(0.9);
            }
            if (gamepad2.b) {
                robot.crane.setGripper(0.1);
            }


                //CHANGE THE GRIPPERS ANGLE

                if (gamepad2.dpad_up) {
                    if (getRuntime() > 0.4) {
                        this.resetRuntime();
                        servoPosAngle += 0.05;
                    }
                    robot.crane.setServoAngle(servoPosAngle);
                }
                if (gamepad2.dpad_down) {
                    if (getRuntime() > 0.4) {
                        this.resetRuntime();
                        servoPosAngle -= 0.05;
                    }
                    robot.crane.setServoAngle(servoPosAngle);
                }

                if (gamepad2.dpad_right) {
                    robot.crane.setServoAngle(0);
                }
                if (gamepad2.dpad_left) {
                    robot.crane.setServoAngle(0.40);
                }

                //GAMEPAD 1
                if (gamepad1.cross) {
                    if (getRuntime() > 0.2) {
                        this.resetRuntime();
                        direction = direction * -1;
                    }
                }

                if (direction == 1) {
                    robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)) * 0.8, calculateThrottle((float) (-gamepad1.left_stick_x)) * 0.8, calculateThrottle((float) (-gamepad1.right_stick_x)) * 0.8));
                } else
                    robot.drive.setDrivePower(new Pose2d(calculateThrottle((gamepad1.left_stick_y)) * 0.8, calculateThrottle((float) (gamepad1.left_stick_x)) * 0.8, calculateThrottle((float) (gamepad1.right_stick_x)) * 0.8));

                if (gamepad2.left_bumper) {
                    telemetry.addLine("a");
                }
//                telemetry.addData("Servo Angle", robot.crane.servoAngle1.getPosition());
//                telemetry.addData("CRANE TICKS LEFT: ", robot.crane.motorCraneLeft.getCurrentPosition());
//                telemetry.addData("CRANE TICKS RIGHT: ", robot.crane.motorCraneRight.getCurrentPosition());
//                telemetry.addData("DIRECTION: ", direction);
//                telemetry.addData("SERVO GRIPPER: ", robot.crane.servoGrippy1.getPosition());
                telemetry.update();
            }
        }
    }



