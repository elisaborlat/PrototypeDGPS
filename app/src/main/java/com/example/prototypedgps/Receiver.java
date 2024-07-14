package com.example.prototypedgps;

import java.util.ArrayList;

public class Receiver {

    private final ArrayList<Observations> epochsArrayList = new ArrayList<>();
    private final Object lock = new Object();
    public Receiver() {
    }

    public ArrayList<Observations> getEpochsArrayList() {
        synchronized (lock) {
            return new ArrayList<>(epochsArrayList);
        }
    }

    public void addEpoch(Observations observation) {
        synchronized (lock) {
            epochsArrayList.add(observation);
        }
    }

}


