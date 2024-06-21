package com.example.prototypedgps;

import java.util.ArrayList;
import java.util.Locale;

public class Receiver {

    private final ArrayList<Observations> epochsArrayList = new ArrayList<>();

    public Receiver() {
    }

    public void addEpoch(Observations observations){
        synchronized (epochsArrayList) {
            epochsArrayList.add(observations);
            System.out.printf(Locale.US,"computeDGPSSingleEpoch| gnssClock base station:" + observations.getRefTime().getMsecGpsTime() + "\n");
        }
    }

    public ArrayList<Observations> getEpochsArrayList() {
        synchronized (epochsArrayList) {
            return new ArrayList<>(epochsArrayList); // Créer une copie pour éviter les modifications concurrentes
        }
    }

}


