package com.example.prototypedgps;

import android.util.Log;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Base64;
import java.util.StringTokenizer;
import java.util.Vector;

public class BaseStation {

    // Point de montage du NTRIPCaster
    private static final String NTRIPCasterMountPoint = "M_04";
//    private static final String NTRIPCasterMountPoint = "ZIM200CHE0";
    static String NTRIPCasterURL = "193.134.218.96";
//    static String NTRIPCasterURL =  "www.euref-ip.net";
//    static String NTRIPCasterURL =  "www.swipos.ch";
    int NTRIPCasterPort = 5001;
//    int NTRIPCasterPort = 2101;

    // Username et password du NTRIPClient
    private final String username = "Ecole01";
//    private static final String username = "HS-HEIG-VD/365";
    private final String password = "365heig";
//    private static final String password = "031270";

    public StationaryAntenna mStationaryAntenna = null;

    private boolean[] bits;
    private boolean[] rollbits;

    private int messagelength = 0;

    private int messageType = 0;
    private boolean headerIsChecked = false;
    // Creation de l'HTTP Basic Authentication Scheme du NTRIPClient
    String userPass = username + ":" + password;
    String encoded = Base64.getEncoder().encodeToString(userPass.getBytes());

    String header = "GET /" + NTRIPCasterMountPoint + " HTTP/1.1\r\n" +
            "Host: " + NTRIPCasterURL + "\r\n" +
            "Ntrip-Version: Ntrip/2.0\r\n" +
            "User-Agent: HEIG-VD SGU NTRIPClient\r\n" +
            "Accept: */*\r\n" +
            "Connection: keep-alive\r\n" +
            "Authorization: Basic " + encoded + "\r\n\r\n";


    NtripCaster NtripCasterHeigM04 = new NtripCaster("M04", "193.134.218.96", 5001, "Ecole01", "365heig");
    NtripCaster NtripCasterHeigYVD2_MSM = new NtripCaster("YVD2_MSM", "193.134.218.96", 5001, "user1", "365heig");
    NtripCaster NtripCasterEuref= new NtripCaster("ZIM200CHE0", "www.euref-ip.net", 2101, "heig01", "356heig");

    NtripCaster NtripCasterSwipos = new NtripCaster("MSM_GIS_GEO_LV95LHN95", "www.swipos.ch", 2101, "HS-HEIG-VD/365", "031270");

    public Receiver getmBaseStationReceiver() {
        return mBaseStationReceiver;
    }

    private final Receiver mBaseStationReceiver;

    public BaseStation(){
        this.mBaseStationReceiver = new Receiver();
    };

    public void getSources() {
        new Thread(new Runnable() {
            @Override
            public void run() {

                String sourceTable = "GET / HTTP/1.0\r\nUser-Agent: NTRIP RTKLIB/2.4.2\r\nAccept: */*\r\nConnection: close\r\n\r\n";

                Socket socket = null;

                try {
                    socket = new Socket(NTRIPCasterURL, NTRIPCasterPort);

                    OutputStream outputStream = socket.getOutputStream();
                    System.out.println("Send to NTRIPCaster :\n" + sourceTable);
                    outputStream.write(sourceTable.getBytes());
                    InputStream inputStream = socket.getInputStream();
                    InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                    BufferedReader bufferedReader = new BufferedReader(inputStreamReader);


                    System.out.println("Etat de connexion du socket : " + socket.isConnected());
                    boolean going = true;
                    boolean first = true;
                    Vector<String> lines = new Vector<String>();
                    // Lire la réponse du serveur
                    while (going) {
                        String newLine = bufferedReader.readLine();
                        if (newLine == null) {
                            going = false;
                        } else if (first) {
                            if (!newLine.equals("SOURCETABLE 200 OK")) {
                                going = false;
                            }
                            first = false;
                        } else {
                            lines.addElement(newLine);
                        }
                    }

                    ArrayList<String> sources = new ArrayList<String>();
                    for (int i = 0; i < lines.size(); i++) {
                        // A new StringTokenizer is created with ";" as delimiter
                        StringTokenizer token = new StringTokenizer(lines.elementAt(i), ";");
                        try {
                            if (token.countTokens() > 1 && token.nextToken().equals("STR")) {
                                //System.out.println(lines.elementAt(i));
                                // We expect the correct source to be the first token after
                                // "STR" to through the token which specifies the RTCM
                                // version
                                // starting with "RTCM "
                                // We haven't seen any specification of the sourcetable, but
                                // according to what we can see from it it should be correct
                                String s = token.nextToken();
                                //while (!s.startsWith("RTCM 3")) {
                                sources.add(s);
                                //	s = token.nextToken();
                                //}
                            }

                            bufferedReader.close();
                            inputStreamReader.close();
                            inputStream.close();
                            socket.close();
                        } catch (IOException e) {
                            throw new RuntimeException(e);
                        }

                    }
                    System.out.println(sources);

                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
            }
        }).start();
    }

    public void registerRtcmMsg(){

        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    // Send the message
                    Socket socket = new Socket(NTRIPCasterURL, NTRIPCasterPort);
                    OutputStream outputStream = socket.getOutputStream();
                    System.out.println("registerRtcmMsg| Send to NTRIPCaster :\n" + header);

                    outputStream.write(header.getBytes());

                    InputStream inputStream = socket.getInputStream();

                    // Read server response
                    while (true) {

                        StringBuilder msgBuilder = new StringBuilder();

                        // Header check
                        if (!headerIsChecked) {
                            // ICY 200 ok
                            int[] correctHeader = {73, 67, 89, 32, 50, 48, 48, 32, 79, 75, 13};

                            int[] headerBuffer = new int[11];
                            for (int i = 0; i < headerBuffer.length; i++) {
                                headerBuffer[i] = inputStream.read();
                                if (headerBuffer[i] != correctHeader[i]) {
//                                    mUiBaseStation.addText("Connexion établie");

                                    break;
                                }
                                headerIsChecked = true;
                            }
                        }

                        // Search message
                        int currentByte = inputStream.read();

                        if (currentByte == -1) {
                            break;
                        }
                        // If get preamble (211 is the byte corresponding to the 8 bits : 11010011)
                        if (currentByte == 211) {

                            //Message length
                            int index;
                            int[] buffer = new int[2];
                            index = 0;
                            buffer[0] = inputStream.read();
                            buffer[1] = inputStream.read();
                            bits = new boolean[buffer.length * 8];

                            rollbits = new boolean[8];
                            for (int k : buffer) {
                                rollbits = Bits.rollByteToBits(k);
                                for (boolean rollbit : rollbits) {
                                    bits[index] = rollbit;
                                    index++;
                                }
                            }
                            messagelength = (int) Bits.bitsToUInt(Bits.subset(bits, 6, 10));


                            // Storage of data message in dataBuffer
                            int index3 = 0;
                            int[] dataBuffer = new int[messagelength]; // storage in bytes
                            boolean[] dataBits = new boolean[dataBuffer.length * 8]; // storage in bits
                            for (int i = 0; i < dataBuffer.length; i++) {
                                dataBuffer[i] = inputStream.read();
                            }
                            rollbits = new boolean[8];
                            for (int k : dataBuffer) {
                                rollbits = Bits.rollByteToBits(k);
                                for (boolean rollbit : rollbits) {
                                    dataBits[index3] = rollbit;
                                    index3++;
                                }
                            }

                            int index2 = 0;
                            int[] MessageTypeBuffer = new int[2];
                            boolean[] MessageTypeBits = new boolean[MessageTypeBuffer.length * 8];

                            rollbits = new boolean[8];
                            for (int i = 0; i < MessageTypeBuffer.length; i++) {
                                rollbits = Bits.rollByteToBits(dataBuffer[i]);
                                for (boolean rollbit : rollbits) {
                                    MessageTypeBits[index2] = rollbit;
                                    index2++;
                                }
                            }

                            messageType = (int) Bits.bitsToUInt(Bits.subset(MessageTypeBits, 0, 12));


                            if (messageType == 1006) {
                                Decode1006Msg mDecode1006Msg = new Decode1006Msg();
                                mStationaryAntenna = mDecode1006Msg.decode(dataBits);
                                System.out.println("registerRtcmMsg| ITRF Realisation Year : " + mStationaryAntenna.getItrl());
                                System.out.println("registerRtcmMsg| ECEF-X : " + mStationaryAntenna.getAntennaRefPointX());
                                System.out.println("registerRtcmMsg| ECEF-Y : " + mStationaryAntenna.getAntennaRefPointY());
                                System.out.println("registerRtcmMsg| ECEF-Z : " + mStationaryAntenna.getAntennaRefPointZ());
                            }

                            if (messageType == 1004) {
                                Decode1004Msg mDecode1004Msg = new Decode1004Msg();
                                Time currentTime = new Time(System.currentTimeMillis());
                                Observations mObservations = mDecode1004Msg.decode(dataBits, currentTime.getGpsWeek());
                                mBaseStationReceiver.addEpoch(mObservations);
                                String rtcmObservationsString = rtcmObservationsToString(mObservations);
                                msgBuilder.append(rtcmObservationsString);

                            }
                        }
                    }
                    outputStream.close();
                    inputStream.close();
                    socket.close();
                } catch (IOException e) {
                    Log.e("TCP", "Error sending message", e);
                }

            }
        }).start();
    }

    private String rtcmObservationsToString(Observations observations) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < observations.getNumSat(); i++) {
            ObservationSet iObservationSet = observations.getSatByID(observations.getSatID(i));
            builder.append("Sat = ").append(iObservationSet.getSatType()).append(observations.getSatID(i)).append(", ");
            builder.append("L2_Pseudorange = ").append(iObservationSet.getPseudorange(1));
            builder.append("\n");
        }
        return builder.toString();
    }

    public RealMatrix getStationaryAntenna() {
        double[][] data = {
                {mStationaryAntenna.getAntennaRefPointX()},
                {mStationaryAntenna.getAntennaRefPointY()},
                {mStationaryAntenna.getAntennaRefPointZ()}
        };
        return new Array2DRowRealMatrix(data);
    }


    private static class NtripCaster{

        private final String NTRIPCasterMountPoint;

        private final String NTRIPCasterURL;
        private final int NTRIPCasterPort;

        // Username et password du NTRIPClient
        private final String username;
        private final String password;

        String userPass;
        String encoded;


        private NtripCaster(String ntripCasterMountPoint, String ntripCasterURL, int ntripCasterPort, String user, String pwd) {
            NTRIPCasterMountPoint = ntripCasterMountPoint;
            NTRIPCasterURL = ntripCasterURL;
            NTRIPCasterPort = ntripCasterPort;
            username = user;
            password = pwd;
            userPass = username + ":" + password;
            encoded = Base64.getEncoder().encodeToString(userPass.getBytes());
        }
        public String getNTRIPCasterMountPoint() {
            return NTRIPCasterMountPoint;
        }

        public String getNTRIPCasterURL() {
            return NTRIPCasterURL;
        }

        public int getNTRIPCasterPort() {
            return NTRIPCasterPort;
        }

        public String getUsername() {
            return username;
        }

        public String getPassword() {
            return password;
        }

        @Override
        public String toString() {
            return "NtripCaster{" +
                    "NTRIPCasterMountPoint='" + NTRIPCasterMountPoint + '\'' +
                    ", NTRIPCasterURL='" + NTRIPCasterURL + '\'' +
                    ", NTRIPCasterPort='" + NTRIPCasterPort + '\'' +
                    ", username='" + username + '\'' +
                    ", password='" + password + '\'' +
                    '}';
        }

    }

    public boolean isStationaryAntennaDecoded() {
        return mStationaryAntenna != null;
    }




}
