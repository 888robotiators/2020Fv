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
    final Float ERROR = 2f;

    // data values from the Jetson
    float xValue;
    float yValue;
    float zValue;
    float roll;
    float pitch;
    float heading;

    public UDPReceiver() {

        try {
            // open a datagram socket to receive messages
            // should be a different port than the sender
            socket = new DatagramSocket(RobotMap.RECEIVER_SOCKET);

            // create a datagram packet to receive data of a certain length
            dat = new DatagramPacket(receiveData, receiveData.length);
        }
        catch (SocketException e) {
            e.printStackTrace();
        }
    }


    @Override
    public void run() {
        receiveMessage();
    }

    public synchronized void receiveMessage() {
        try {
            socket.receive(dat);

            ByteBuffer bbuf = ByteBuffer.wrap(dat.getData());
            xValue = bbuf.getFloat(0);
            zValue = bbuf.getFloat(4);
            yValue = bbuf.getFloat(8);
            roll = bbuf.getFloat(12);
            pitch = bbuf.getFloat(16);
            heading = bbuf.getFloat(20);
            // heading = Math.atan2((double) XValue, (double) YValue) *
            // (180/Math.PI);

            SmartDashboard.putNumber("X Value", (double) xValue);
            SmartDashboard.putNumber("Y Value", (double) yValue);
            SmartDashboard.putNumber("Z Value", (double) zValue);
            SmartDashboard.putNumber("Roll", (double) roll);
            SmartDashboard.putNumber("Pitch", (double) pitch);
            SmartDashboard.putNumber("Yaw", (double) heading);
            // SmartDashboard.putNumber("line heading", (double) heading);
            if ((int) xValue != -99 && (int) yValue != -99 && (int) zValue != -99) {

                SmartDashboard.putBoolean("vision", true);
            }
            else {
                SmartDashboard.putBoolean("vision", false);
            }

        }
        catch (Exception e) {
            e.printStackTrace();
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
    
    public synchronized double[] getTarget() {

        return new double[] { xValue, yValue, zValue, roll, pitch, heading };
    }

    public synchronized Position getTargetPosition() {
        return new Position(xValue, yValue, zValue, heading, roll, pitch);
    }
    /*
    public synchronized double getTarget() {
        return zValue;
        
    }
    */
}
