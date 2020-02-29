/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;

import disc.data.Position;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UDPReceiver implements Runnable {

    // declare objects for receiving data
    DatagramSocket socket;
    DatagramPacket dat;
    byte[] receiveData = new byte[24]; // data should be 4 floats in
                                       // length

    Position targetPos;

    // data values from the Jetson
    float xValue = -99f;
    float yValue = -99f;
    float zValue = -99f;
    float roll = -99f;
    float pitch = -99f;
    float heading = -99f;

    public UDPReceiver() {

        try {
            // open a datagram socket to receive messages
            // should be a different port than the sender
            socket = new DatagramSocket(RobotMap.RECEIVER_SOCKET);
            socket.setSoTimeout(1);

            // create a datagram packet to receive data of a certain length
            dat = new DatagramPacket(receiveData, receiveData.length);
        }
        catch (SocketException e) {
            e.printStackTrace();
        }

        targetPos = new Position(xValue, yValue, zValue, heading, roll, pitch);
    }

    @Override
    public void run() {
        receiveMessage();
    }

    public void receiveMessage() {
        while (true) {
        try {
            socket.receive(dat);

            ByteBuffer bbuf = ByteBuffer.wrap(dat.getData());
            xValue = bbuf.getFloat(0);
            zValue = bbuf.getFloat(4);
            yValue = bbuf.getFloat(8);
            roll = bbuf.getFloat(12);
            pitch = bbuf.getFloat(16);
            heading = bbuf.getFloat(20);

            // SmartDashboard.putNumber("X Value", (double) xValue);
            // SmartDashboard.putNumber("Y Value", (double) yValue);
            // SmartDashboard.putNumber("Z Value", (double) zValue);
            // SmartDashboard.putNumber("Roll", (double) roll);
            // SmartDashboard.putNumber("Pitch", (double) pitch);
            // SmartDashboard.putNumber("Yaw", (double) heading);
            // SmartDashboard.putNumber("line heading", (double) heading);
            if ((int) xValue != -99 && (int) yValue != -99
                    && (int) zValue != -99) {

                SmartDashboard.putBoolean("vision", true);
            }
            else {
                SmartDashboard.putBoolean("vision", false);
            }

        }
        catch (Exception e) {
            e.printStackTrace();
        }

        targetPos.setX(xValue);
        targetPos.setY(yValue);
        targetPos.setZ(zValue);
        targetPos.setHeading(heading);
        targetPos.setPitch(pitch);
        targetPos.setRoll(roll);

    }
    }

    public boolean hasVision() {
        if ((int) xValue != -99 && (int) yValue != -99 && (int) zValue != -99) {
            return true;
        }
        else {
            return false;
        }
    }

    public synchronized Position getTargetPosition() {
        return targetPos;
    }
    /*
     * public synchronized double getTarget() { return zValue; }
     */
}
