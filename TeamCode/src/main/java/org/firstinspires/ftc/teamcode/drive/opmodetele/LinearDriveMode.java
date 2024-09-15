package org.firstinspires.ftc.teamcode.drive.opmodetele;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.robot.Robot;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    int direction = 1; //daca e true e in fata daca e false e in spate
    double servoPosSlides = 0.5;
    double servoPosGrippy = 0;
    // EXPONENTIAL THROTTLE
    public double calculateThrottle(float x) {
        int sign = -1;
        if (x > 0) sign = 1;
        return sign * 3 * abs(x);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }
        // INIT CODE
        telemetry.addData(">", "Initialized");
        telemetry.update();



        waitForStart();
        if (isStopRequested()) return;

            // TELEOP CODE

        while (opModeIsActive()) {

            // GAMEPAD 2

            //EXTEND AND RETRACT SLIDES
            if (gamepad2.left_bumper) {
                servoPosSlides = -1;
                robot.crane.moveSlides(servoPosSlides);
            } else if (gamepad2.right_bumper) {
                servoPosSlides = 1;
                robot.crane.moveSlides(servoPosSlides);
            } else {
                servoPosSlides = 0;
                robot.crane.moveSlides(servoPosSlides);
            }

            //MOVE THE ENTIRE CRANE

            if(gamepad2.left_trigger > 0.1){
                robot.crane.craneTarget -= (int) calculateThrottle(gamepad1.left_trigger);
            }
            else if(gamepad2.right_trigger > 0.1){
                robot.crane.craneTarget += (int) calculateThrottle(gamepad1.right_trigger);
            }
            robot.crane.motorCrane1.setPower(robot.crane.cranePower(robot.crane.craneTarget));
            robot.crane.motorCrane2.setPower(robot.crane.cranePower(robot.crane.craneTarget));

            //OPEN AND CLOSE THE GRIPPER
            if (gamepad2.a) {
                robot.crane.setGripper(0.9);
            }
            if (gamepad2.b) {
                robot.crane.setGripper(0.1);
            }

            // GAMEPAD 1

            //CHANGE THE DIRECTION OF THE MECANUM DRIVETRAIN
                if (gamepad1.cross) {
                    if (getRuntime() > 0.2) {
                        this.resetRuntime();
                        direction = direction * -1;
                    }
                }

                if (direction == 1) {
                    robot.drive.setDrivePower(new Pose2d(calculateThrottle((gamepad1.left_stick_y)) * 0.8, calculateThrottle((float) (-gamepad1.left_stick_x)) * 0.8, calculateThrottle((float) (-gamepad1.right_stick_x)) * 0.8));
                } else
                    robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)) * 0.8, calculateThrottle((float) (gamepad1.left_stick_x)) * 0.8, calculateThrottle((float) (gamepad1.right_stick_x)) * 0.8));





//                telemetry.addData("CRANE TICKS LEFT: ", robot.crane.motorCraneLeft.getCurrentPosition());
//                telemetry.addData("CRANE TICKS RIGHT: ", robot.crane.motorCraneRight.getCurrentPosition());
//                telemetry.addData("DIRECTION: ", direction);
//                telemetry.addData("SERVO GRIPPER: ", robot.crane.servoGrippy1.getPosition());
                telemetry.update();
            }

        }

    }



