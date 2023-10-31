package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeArm {
    public Telemetry telemetry;
    public OuttakeArm(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("=============");
    }
}
