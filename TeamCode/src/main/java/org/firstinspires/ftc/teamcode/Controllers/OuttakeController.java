package org.firstinspires.ftc.teamcode.Controllers;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;

public class OuttakeController {
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;
    LinearOpMode currentOpMode;


    public OuttakeController(OuttakeSlides outtakeSlides, OuttakeArm outtakeArm, LinearOpMode currentOpMode){
        this.outtakeSlides = outtakeSlides;
        this.outtakeArm = outtakeArm;
        this.currentOpMode = currentOpMode;
    }

    public void moveTransferToPickup(){
        outtakeArm.closeGrip();
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.PICKUP);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.PICKUP);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.PICKUP);
    }

    public void movePickupToTransfer(){
        outtakeArm.closeGrip();
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.TRANSFER);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
    }

    public void moveTransferToReadyForTransfer(){
        outtakeArm.closeGrip();
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
        safeWaitMilliSeconds(100);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
    }

    public void moveReadyForTransferToTransfer(){
        outtakeArm.openGrip();
        if (outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER) &&
                outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER)) {
            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
            outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.TRANSFER);
        } else {
            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
            outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
        }
    }

    public void moveTravelToReadyForTransfer(){
        outtakeArm.closeGrip();
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
        safeWaitMilliSeconds(200);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
        safeWaitMilliSeconds(100);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
    }

    public void moveReadyForTransferToDropLevel(OuttakeSlides.OUTTAKE_SLIDE_STATE outtakeSlideStateDropLevel){
        outtakeArm.closeGrip();
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP);
        if(outtakeSlideStateDropLevel == OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LEVEL_LOW ||
            outtakeSlideStateDropLevel == OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_BELOW_LOW) {
            safeWaitMilliSeconds(100);
        }
        outtakeSlides.moveOuttakeSlides(outtakeSlideStateDropLevel);
    }

    public void moveDropToTravel(){
        outtakeArm.closeGrip();
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRAVEL);
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.TRAVEL);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRAVEL);
    }

    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!currentOpMode.isStopRequested() && timer.time() < time) {
        }
    }

}
