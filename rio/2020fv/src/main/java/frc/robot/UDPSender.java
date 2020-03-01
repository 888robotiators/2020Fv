package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;

/**
 * Looks for raw UDP messages
 */
public class UDPSender extends Thread {

    // declare the objects for sending data
    private static DatagramSocket sock;
    private DatagramPacket message;
    private InetAddress jetsonAddress;

    int cycle = 0;

    byte[] address = { 10, 8, 88, 19 }; // Jetson
                                        // IP
                                        // address

    byte[] byteCameraMessage = ByteBuffer.allocate(16).putInt(cycle).array();

    /**
     * Declares the socket and empty message
     */
    public UDPSender() {

        try {
            // creates an InetAdress to represent the IP adress of the Jetson
            jetsonAddress = InetAddress.getByAddress(address);

            // opens datagram socket to send messages
            sock = new DatagramSocket(5806);

            // creates datagram packet to receive messages of a certain length
            // from an address and port
            // port should be the same as sender port on jetson
            message = new DatagramPacket(byteCameraMessage,
                    byteCameraMessage.length, jetsonAddress, 5809);

        }
        catch (Exception e) {

        }
    }

    /**
     * Tries to send a message to the Jetson
     */
    public void sendMessage() {
        try {
            sock.send(message);

        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }
}