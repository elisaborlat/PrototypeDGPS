package com.example.pseudorange;

import android.location.GnssNavigationMessage;
import android.location.cts.nano.Ephemeris;

import java.io.FileOutputStream;
import java.io.IOException;

public class EphemerisManager {


        private Ephemeris.GpsNavMessageProto mHardwareGpsNavMessageProto = null;

        private final GpsNavigationMessageStore mGpsNavigationMessageStore = new GpsNavigationMessageStore();

        public void parseHwNavigationMessageUpdates(GnssNavigationMessage navigationMessage){

            byte messagePrn = (byte) navigationMessage.getSvid();
            byte messageType = (byte) (navigationMessage.getType() >> 8);
            int subMessageId = navigationMessage.getSubmessageId();
            byte[] messageRawData = navigationMessage.getData();

            // Parse only GPS navigation messages for now
            if (messageType == 1) {
                mGpsNavigationMessageStore.onNavMessageReported(
                        messagePrn, messageType, (short) subMessageId, messageRawData);
                mHardwareGpsNavMessageProto = mGpsNavigationMessageStore.createDecodedNavMessage();
            }
        }

        public Ephemeris.GpsNavMessageProto getmHardwareGpsNavMessageProto() {
            return mHardwareGpsNavMessageProto;
        }

        public void saveDataToFile(FileOutputStream fos) throws IOException {
            String text = String.valueOf(getmHardwareGpsNavMessageProto());
            if(!text.equals("null")){
                fos.write(text.getBytes());
            }
            fos.close();
        }

        public void loadLastEph(StringBuilder sb){

            //Read sb
            mHardwareGpsNavMessageProto = mGpsNavigationMessageStore.loadLastEph(sb);

        }
    }

