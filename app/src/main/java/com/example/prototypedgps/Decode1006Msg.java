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

public class Decode1006Msg {

    public StationaryAntenna decode(boolean[] bits){

        int start = 12;

        StationaryAntenna stationaryAntenna = new StationaryAntenna();

        stationaryAntenna.setStationID((int)Bits.bitsToUInt(Bits.subset(bits, start, 12)));
        start += 12;
        stationaryAntenna.setItrl((int)Bits.bitsToUInt(Bits.subset(bits, start, 6)));
        start += 6;
        stationaryAntenna.setGpsIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        stationaryAntenna.setGlonassIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        stationaryAntenna.setRgalileoIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        stationaryAntenna.setRstationIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        // x coordinate
        stationaryAntenna.setAntennaRefPointX(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
        start += 38;
        stationaryAntenna.setSreceiverOscillator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        stationaryAntenna.setReserved1((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
        start += 1;
        // y coordinate
        stationaryAntenna.setAntennaRefPointY(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
        start += 38;
        stationaryAntenna.setReserved2((int)Bits.bitsToUInt(Bits.subset(bits, start, 2)));
        start += 2;
        // 5 coordinate
        stationaryAntenna.setAntennaRefPointZ(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
        start += 38;
        stationaryAntenna.setAntennaHeight(Bits.bitsToUInt(Bits.subset(bits, start, 16)) * 0.0001);

        return stationaryAntenna;
    }

}
