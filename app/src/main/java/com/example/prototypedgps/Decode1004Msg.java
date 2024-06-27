package com.example.prototypedgps;

/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 *
 * goGPS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * goGPS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with goGPS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

public class Decode1004Msg {

    public Observations decode(boolean[] bits, int week) {

        if (bits.length < 64)
            return null;

        // Message Header
        int start = 12;
        int DF003 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 12)); //Reference Station ID
        start += 12;
        double DF004 = (double)Bits.bitsToUInt(Bits.subset(bits, start, 30)); //GPS Epoch Time(TOW)
        start += 30;
        boolean DF005 = (Bits.bitsToUInt(Bits.subset(bits, start, 1))==1); //Synchronous GNSS Flag
        start += 1;
        int DF006 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 5)); //No. of GPS Satellite Signals Processed
        start += 5;
        boolean DF007 = (Bits.bitsToUInt(Bits.subset(bits, start, 1))==1); //GPS Divergence-free Smoothing Indicator
        start += 1;
        int DF008 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 3)); // GPS Smoothing Interval
        start += 3;


        if (bits.length < 125*DF006)
            return null;

        Observations o = new Observations(new Time(week,DF004/1000),0);


        // Iteration on satellite at current epoch
        for (int i = 0; i < DF006 ; i++) {
            int DF009 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 6));
            start += 6;
            boolean DF010 = (Bits.bitsToUInt(Bits.subset(bits, start, 1))==1); // GPS L1 Code Indicator (0: C/A Code, 1: P(Y) Code Direct)
            start += 1;
            long DF011 = Bits.bitsToUInt(Bits.subset(bits,start, 24));
            start += 24;
            long DF012 = Bits.bitsTwoComplement(Bits.subset(bits, start, 20));
            start += 20;
            int DF013 = (int)Bits.bitsToUInt(Bits.subset(bits, start,7));
            start += 7;
            int DF014 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 8));
            start += 8;
            int DF015 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 8));
            start += 8;
            int DF016 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 2)); // GPS L2 Code Indicator
            start += 2;
            int DF017 = (int)Bits.bitsTwoComplement(Bits.subset(bits,start, 14));
            start += 14;
            long DF018 = Bits.bitsTwoComplement(Bits.subset(bits,start, 20));
            start += 20;
            int DF019 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 7));
            start += 7;
            int DF020 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 8));
            start += 8;

            ObservationSet os = new ObservationSet();
            os.setSatID(DF009);
            os.setSatType('G');

            //// L1

            // Pseudorange
            double DF011d=DF011*0.02+DF014*299792.458;

            if (DF012 != 0x80000) { // hexadecimal representation
                if(DF010){
                    os.setCodeP(ObservationSet.L1, DF011d);
                }else{
                    os.setCodeC(ObservationSet.L1, DF011d);
                }
                double cp1 = DF012*0.0005/(Constants.SPEED_OF_LIGHT/Constants.FL1);

                os.setPhaseCycles(ObservationSet.L1, DF011d/(Constants.SPEED_OF_LIGHT/Constants.FL1)+cp1);
            }

            // CNR
            double snr = (DF015 * 0.25);
            if (snr < 0.0 || 63.75 < snr) {
                snr = 0.0;
            }
            //snr = (snr<=0.0||255.5<=snr)?0.0:snr+0.5;
            os.setSignalStrength(ObservationSet.L1, (float)snr);


            //// L2
            if (DF017!=0x2000) {
                os.setCodeP(ObservationSet.L2, DF011d+DF017*0.02);
            }
            if (DF018!=0x80000) {
                double cp2=DF018*0.0005/(Constants.SPEED_OF_LIGHT/Constants.FL2);

                os.setPhaseCycles(ObservationSet.L2,DF011d/(Constants.SPEED_OF_LIGHT/Constants.FL2)+cp2);
            }

            //os.setPseudorangeCode(ObservationSet.L2, ((DF011 + DF017) * 0.02) + (DF014 * 299792.458));
            //os.setPhase(ObservationSet.L2,(os.getCoarseAcquisition() + (DF018*0.0005)) / Constants.LAMBDA_2);

            snr = (DF020 * 0.25);
            if (snr < 0.0 || 63.75 < snr){
                snr = 0.0;
            }
            os.setSignalStrength(ObservationSet.L2, (float)snr);
            o.setGps(i, os);
        }

        //client.addObservation(o);
        return o;
    }

}
