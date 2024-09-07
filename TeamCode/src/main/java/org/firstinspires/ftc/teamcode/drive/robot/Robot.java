package org.firstinspires.ftc.teamcode.drive.robot;


import android.graphics.Camera;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.subsystems.Crane;


public class Robot {
    private boolean initialize;
    public SampleMecanumDrive drive;
    public Crane crane;


    public Robot(HardwareMap hardwareMap){
        initialize = true;
        drive = new SampleMecanumDrive(hardwareMap);
        crane = new Crane(hardwareMap);
        initialize = false;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean isInitialize() {return initialize;}
}