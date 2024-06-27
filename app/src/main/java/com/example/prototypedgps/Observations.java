package com.example.prototypedgps;

/*
 * Copyright (c) 2010, Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
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
 *
 */

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.ArrayList;

/**
 * Observations class
 * @author Eugenio Realini, Cryms.com
 */
public class Observations implements Streamable {

    private final static int STREAM_V = 1;

    private Time refTime; /* Reference time of the dataset */
    private int eventFlag; /* Event flag */

    private ArrayList<ObservationSet> obsSet; /* sets of observations */


    public Observations(Time time, int flag){
        this.refTime = time;
        this.eventFlag = flag;
    }


    public int getNumSat(){
        if(obsSet == null) return 0;
        int nsat = 0;
        for(int i=0;i<obsSet.size();i++)
            if(obsSet.get(i)!=null) nsat++;
        return nsat;
    }

    public ObservationSet getSatByIdx(int idx){
        return obsSet.get(idx);
    }

    public ObservationSet getSatByID(Integer satID){
        if(obsSet == null || satID==null) return null;
        for(int i=0;i<obsSet.size();i++)
            if(obsSet.get(i)!=null && obsSet.get(i).getSatID()== satID) return obsSet.get(i);
        return null;
    }


    public Integer getSatID(int idx){
        return getSatByIdx(idx).getSatID();
    }


    /**
     * @return the refTime
     */
    public Time getRefTime() {
        return refTime;
    }


    public void setGps(int i, ObservationSet os ){
        if(obsSet==null) obsSet = new ArrayList<>(i + 1);
        if(i==obsSet.size()){
            obsSet.add(os);
        }else{
            int c=obsSet.size();
            while(c++<=i) obsSet.add(null);
            obsSet.set(i,os);
        }
    }

    public int write(DataOutputStream dos) throws IOException{
        dos.writeUTF(MESSAGE_OBSERVATIONS); // 5
        dos.writeInt(STREAM_V); // 4
        dos.writeLong(refTime==null?-1:refTime.getMsec()); // 13
        dos.writeDouble(refTime==null?-1:refTime.getFraction());
        dos.write(eventFlag); // 14
        dos.write(obsSet==null?0:obsSet.size()); // 15
        int size=19;
        if(obsSet!=null){
            for(int i=0;i<obsSet.size();i++){
                size += ((ObservationSet)obsSet.get(i)).write(dos);
            }
        }
        return size;
    }


    /* (non-Javadoc)
     * @see org.gogpsproject.Streamable#read(java.io.DataInputStream)
     */
    @Override
    public void read(DataInputStream dai, boolean oldVersion) throws IOException {
        int v=1;
        if(!oldVersion) v=dai.readInt();

        if(v==1){
            refTime = new Time(dai.readLong(), dai.readDouble());
            eventFlag = dai.read();
            int size = dai.read();
            obsSet = new ArrayList<>(size);

            for(int i=0;i<size;i++){
                if(!oldVersion) dai.readUTF();
                ObservationSet os = new ObservationSet(dai, oldVersion);
                obsSet.add(os);
            }
        }else{
            throw new IOException("Unknown format version:"+v);
        }
    }


    public ArrayList<ObservationSet> getObsSet() {
        return obsSet;
    }
}
