package org.firstinspires.ftc.teamcode.drive.opmodetele;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    int direction = 1; //daca e true e in fata daca e false e in spate
    double servoPosSlides = 0.5;
    double servoPosGrippy = 0;
    double servoPosAngle = 0.5;

    public double calculateThrottle(float x){
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







        //TELEOP CODE

        //GAMEPAD 2

        //EXTEND AND RETRACT SLIDES
        if (gamepad2.left_trigger > 0.1) {
            if(System.nanoTime() > 2000000000){
                this.resetRuntime();
                servoPosSlides -= 0.05;
            }
            robot.crane.moveSlides(servoPosSlides);
        }
        if (gamepad2.right_trigger > 0.1) {
            if(System.nanoTime() > 2000000000){
                this.resetRuntime();
                servoPosSlides += 0.05;
            }
        }

        //MOVE THE ENTIRE CRANE
        if(gamepad2.left_bumper){
            robot.crane.motorCrane1.setPower(-1);
            robot.crane.motorCrane2.setPower(-1);
        }
        if(gamepad2.right_bumper){
            robot.crane.motorCrane1.setPower(1);
            robot.crane.motorCrane2.setPower(1);
        }

        //OPEN AND CLOSE THE GRIPPER
        if(gamepad2.cross){
            if(System.nanoTime() > 2000000000){
                this.resetRuntime();
                servoPosGrippy += 0.05;
            }
            robot.crane.setGripper(servoPosSlides);
        }
        if(gamepad2.circle){
            if(System.nanoTime() > 2000000000){
                this.resetRuntime();
                servoPosGrippy -= 0.05;
            }
            robot.crane.setGripper(servoPosSlides);

        //CHANGE THE GRIPPERS ANGLE
        }
        if(gamepad2.dpad_up){
            if(System.nanoTime() > 2000000000){
                this.resetRuntime();
                servoPosAngle += 0.05;
            }
            robot.crane.setServoAngle(servoPosAngle);
        }
        if(gamepad2.dpad_down){
            if(System.nanoTime() > 2000000000){
                this.resetRuntime();
                servoPosAngle -= 0.05;
            }
            robot.crane.setServoAngle(servoPosAngle);
        }

        //GAMEPAD 1
        if(gamepad1.cross){
            direction = direction * -1;
        }

        if(direction == 1) {
            robot.drive.setDrivePower(new Pose2d(calculateThrottle((-gamepad1.left_stick_y)) * 0.8, calculateThrottle((float) (-gamepad1.left_stick_x)) * 0.8, calculateThrottle((float) (-gamepad1.right_stick_x)) * 0.8));
        }
        else robot.drive.setDrivePower(new Pose2d(calculateThrottle((gamepad1.left_stick_y)) * 0.8, calculateThrottle((float) (gamepad1.left_stick_x)) * 0.8, calculateThrottle((float) (gamepad1.right_stick_x)) * 0.8));


    }
}
