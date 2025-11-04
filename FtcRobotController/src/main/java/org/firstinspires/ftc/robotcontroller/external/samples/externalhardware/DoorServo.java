package org.firstinspires.ftc.robotcontroller.external.samples.externalhardware;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
public class DoorServo {
    public Servo servo;
    public double SetPos = 0;
    public void init() {
        servo = hardwareMap.get(Servo.class, "Door");
//        servo = hardwareMap.get(CRServo.class, "servo");
    }
    public void loop() {

    }

}
