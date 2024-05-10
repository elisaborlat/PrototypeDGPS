package com.example.prototypedgps;

import java.util.ArrayList;

public class Receiver {

    private final ArrayList<Observations> epochsArrayList = new ArrayList<>();

    public Receiver() {
    }

    public void addEpoch(Observations observations){
        epochsArrayList.add(observations);
    }

    public ArrayList<Observations> getEpochsArrayList() {
        return epochsArrayList;
    }

}
