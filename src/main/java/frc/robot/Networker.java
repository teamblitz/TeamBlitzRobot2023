package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import java.io.IOException;
import java.net.*;
import java.nio.ByteBuffer;

public class Networker extends Thread {

    private byte[] buf = new byte[256];
    private DatagramSocket socket;
    private InetAddress address = InetAddress.getByName("10.20.83.130");

    private final int port = 5810;

    public Networker() throws IOException {}

    @Override
    public void run() {
        try {
            socket = new DatagramSocket(5810);
        } catch (SocketException e) {
            System.out.println(e);
        }
        while (true) {
            buf = ByteBuffer.allocate(Long.BYTES).putLong(RobotController.getFPGATime()).array();
            DatagramPacket packet = new DatagramPacket(buf, buf.length, address, port);
            try {
                socket.send(packet);
                Thread.sleep(5*1000);
            } catch (IOException e) {
                System.out.println(e);
            } catch (InterruptedException e) {
                System.out.println(e);
                break;
            }
        }
    }
}
