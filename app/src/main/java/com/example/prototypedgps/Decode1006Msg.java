package com.example.prototypedgps;

public class Decode1006Msg {

    public StationaryAntenna decode(boolean[] bits){

        int start = 12;

        StationaryAntenna stationaryantenne = new StationaryAntenna();

        stationaryantenne.setStationID((int)Bits.bitsToUInt(Bits.subset(bits, start, 12)));
        start += 12;
        stationaryantenne.setItrl((int)Bits.bitsToUInt(Bits.subset(bits, start, 6)));
        start += 6;
        stationaryantenne.setGpsIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        stationaryantenne.setGlonassIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        stationaryantenne.setRgalileoIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        stationaryantenne.setRstationIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        //System.out.println("x"+Bits.bitsToStr(Bits.subset(bits, start, 38)));
        stationaryantenne.setAntennaRefPointX(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
        start += 38;
        stationaryantenne.setSreceiverOscillator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        stationaryantenne.setReserved1((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        //System.out.println("y"+Bits.bitsToStr(Bits.subset(bits, start, 38)));
        stationaryantenne.setAntennaRefPointY(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
        start += 38;
        stationaryantenne.setReserved2((int)Bits.bitsToUInt(Bits.subset(bits, start, 2)));
        start += 2;
        //System.out.println("z"+Bits.bitsToStr(Bits.subset(bits, start, 38)));
        stationaryantenne.setAntennaRefPointZ(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
        start += 38;
        stationaryantenne.setAntennaHeight(Bits.bitsToUInt(Bits.subset(bits, start, 16)) * 0.0001);
        start += 16;

        return stationaryantenne;
    }

}
