package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagsExample extends OpMode {

AprilTagsWebCam aprilTagsWebCam = new AprilTagsWebCam();

    @Override
    public void init(){
    aprilTagsWebCam.init(hardwareMap, telemetry);

    }

    @Override
    public void loop(){
//update the vision portal
        aprilTagsWebCam.update();
        AprilTagDetection id20 = aprilTagsWebCam.getTagBySpecificId(20);
        telemetry.addData("id20 String", id20.toString());
//

    }


}


