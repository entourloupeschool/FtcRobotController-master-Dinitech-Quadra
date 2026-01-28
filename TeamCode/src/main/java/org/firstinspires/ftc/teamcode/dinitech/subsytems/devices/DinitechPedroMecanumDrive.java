package org.firstinspires.ftc.teamcode.dinitech.subsytems.devices;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.FOLLOWER_T_POSITION_END;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DinitechPedroMecanumDrive {
    private final Follower follower;

    /**
     * The constructor for the DinitechPedroMecanumDrive class.
     * @param hardwareMap The hardware map for accessing robot hardware.
     * @param beginPose The initial pose of the robot.
     */
    public DinitechPedroMecanumDrive(HardwareMap hardwareMap, Pose beginPose){
        this.follower = DinitechFollower.createFollower(hardwareMap);
        setStartingPose(beginPose);
        update();
    }

    public void update(){
        //Call this once per loop
        follower.update();
    }

    /**
     * setDrivePowers to the follower
     * @param forward the forward power
     * @param strafe the strafe power
     * @param turn the turn power
     */
    public void setDrivePowers(double forward, double strafe, double turn, boolean robotCentric) {
        follower.setTeleOpDrive(forward, strafe, turn, robotCentric);
    }

    public Pose getPose(){
        return follower.getPose();
    }
    public double getHeading(){
        return follower.getPose().getHeading();
    }

    public double getPoseX(){
        return follower.getPose().getX();
    }
    public double getPoseY(){
        return follower.getPose().getY();
    }


    public void startTeleOpDrive(){
        follower.startTeleOpDrive();
    }

    public void followPathChain(PathChain path, double maxPower, boolean holdEnd){
        follower.followPath(path, maxPower, holdEnd);
    }

    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    public void setHeading(double heading){
        follower.setHeading(heading);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public boolean isOnPath(){
        return follower.getCurrentPath() != null;
    }

    public boolean isPathQuasiDone(){
//        boolean isVelocityLow = follower.getVelocity().getMagnitude() < follower.getCurrentPath().getPathEndVelocityConstraint();
//        boolean isHeadingSufficient = follower.getAngularVelocity() < 0.055;
//        boolean isAtParametricEnd = follower.getCurrentPath().isAtParametricEnd();
//        return isHeadingSufficient && isAtParametricEnd;
        return follower.getCurrentTValue() > FOLLOWER_T_POSITION_END;
    }

    public void setMaxPower(double globalMaxPower) {
        follower.setMaxPower(globalMaxPower);
    }

    public PathBuilder getPathBuilder() {
        return follower.pathBuilder();
    }

    public Follower getFollower(){
        return follower;
    }
}
