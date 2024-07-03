package com.example.prototypedgps;

import org.json.JSONObject;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;


public class UDPSender {

    private final String ipAddress;
    private final int port;
    private DatagramSocket socket;

    public UDPSender(String ipAddress, int port) {
        this.ipAddress = ipAddress;
        this.port = port;
        try {
            this.socket = new DatagramSocket();
        } catch (Exception e) {
            System.err.println(e.getMessage());
        }
    }

    public void sendMessage(String message) {
        try {
            InetAddress address = InetAddress.getByName(ipAddress);
            byte[] buffer = message.getBytes();
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length, address, port);
            socket.send(packet);
        } catch (Exception e) {
            System.err.println(e.getMessage());
        }
    }

    public void sendJSONObject(JSONObject json) {
        try {
            InetAddress address = InetAddress.getByName(ipAddress);
            byte[] buffer = json.toString().getBytes();
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length, address, port);
            socket.send(packet);
        } catch (Exception e) {
            System.err.println(e.getMessage());
        }
    }

    public void close() {
        if (socket != null && !socket.isClosed()) {
            socket.close();
        }
    }

}
