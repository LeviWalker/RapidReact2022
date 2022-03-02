package frc.robot.vision;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.InputStreamReader;
import java.net.Socket;

/**
 * Note: This may or may not be the most up-to-date model of the VisionClient code.
 *       It may need to be updated before and as we test it.
 */
public class VisionClient extends Thread {
    /**
     * Number of values in the data string
     */
    private static final byte NumValues = 3;

    /**
     * Socket to create connection
     */
    private Socket socket;
    /**
     * reader to read data sent
     */
    private BufferedReader input;
    /**
     * output stream to send data request
     */
    private DataOutputStream output;

    // indicies for values

    /**
     * Index of Distance value
     */
    private static final int kDistanceIndex = 0;

    /**
     * Index of Angle value
     */
    private static final int kAngleIndex = 1;

    /**
     * Array for holding data values
     * 
     * If you're not familiar with the "volatile" keyword,
     * it basically tells Java that you want this data
     * stored in the main memory. This makes "volatile"
     * variables accessible across different threads, but
     * accessing data in the main memory takes longer.
     * For this reason, "volatile" should be used sparingly,
     * only in cases where it is absolutely required.
     * 
     * @see {@link VisionClient#kDistanceIndex}
     * @see {@link VisionClient#AngleIndex}
     */
    private volatile double[] dataArray;

    /**
     * Creates a vision client with the following IP Address and port
     * @param ipAddress IP address
     * @param port Port
     * @throws VisionClientException you can print the stack trace to see where the error is
     */
    public VisionClient(String ipAddress, int port) throws VisionClientException {

        // try to establish connection with server
        boolean waitForConnection = true;
        while (waitForConnection) {
            try {
                socket = new Socket(ipAddress, port);
                waitForConnection = false;
            } catch (Exception ignored) {
                throw new VisionClientException("Location: Socket");
            }     
        }

        // try to establish reader connection on the socket
        waitForConnection = true;
        while (waitForConnection) {
            try {
                input = new BufferedReader(
                    new InputStreamReader(socket.getInputStream())
                );
                waitForConnection = false;
            } catch (Exception ignored) {
                throw new VisionClientException("Location: Input Reader");
            }
        }

        // try to establish output connection on the socket
        waitForConnection = true;
        while (waitForConnection) {
            try {
                output = new DataOutputStream(socket.getOutputStream());
                waitForConnection = false;
            } catch (Exception ignored) {
                throw new VisionClientException("Location: Output Stream");
            }
        }
        
        // initialize data array to have NumValues values
        dataArray = new double[NumValues];
    }

    /**
     * Thread method that will run during the lifetime of the thread
     */
    @Override
    public void run() {
        // loop forever
        while (true) {
            try {
                // send the server a message so that it will respond with data
                output.writeBytes("value\n");
                // get the data from the TCP connection and split it into a String array
                String[] data = input.readLine().split(",");
                // parse the String array into the double array
                synchronized (this) {
                    for (int i = 0; i < NumValues; i++)
                        dataArray[i] = Double.parseDouble(data[i]);
                }
            } catch (Exception e) {
                // In case of a runtime exception, this will send a message to the console
                e.printStackTrace();
            }
        }
    }

    /**
     * Method for getting the values from the data array
     * @param id the index
     * @return the value of the data at the given index
     * @see {@link VisionClient#kDistanceIndex}
     * @see {@link VisionClient#kAngleIndex}
     */
    public synchronized double getValue(byte id) {
        return dataArray[id];
    }

    /**
     * Method for getting the distance value from the data array
     * @return the distance value
     */
    public synchronized double getDistance() {
        return dataArray[kDistanceIndex];
    }

    /**
     * Method for getting the angle value from the data array
     * @return the angle value
     */
    public synchronized double getAngle() {
        return dataArray[kAngleIndex];
    }

    /**
     * Method for getting the data array
     * @return the data array
     */
    public synchronized double[] getDataArray() {
        return dataArray;
    }

    /**
     * @return A String that will describe the values from the vision server.
     *         You can just print this out with {@link System.out#println()}
     */
    @Override
    public String toString() {
        return "Distance: " + this.getDistance() + ",\t\tAngle: " + this.getAngle();
    }

    /**
     * This class has been implemented for better testing and calibration purposes only and
     * should be removed once the vision client is fully operational due to its over-complex nature.
     */
    @SuppressWarnings("serial")
    public static final class VisionClientException extends Exception {
        public VisionClientException(String message) {
            super("Failed to initialize the Vision Client.\n" + message);
        }
    }
}