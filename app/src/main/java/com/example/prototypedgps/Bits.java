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

public class Bits {

    public static long bitsToUInt(boolean[] bits) {
        long result = 0;
        long pow2 = 1;
        for (int i = bits.length-1; i >= 0; i--) {
            if (bits[i]) {
                result = result + pow2 ;//(int) java.lang.Math.pow(2, (bits.length - i - 1));
            }
            pow2 = pow2 * 2;
        }

        return result;
    }


    public static long bitsTwoComplement(boolean[] bits) {
        long result;

        if (!bits[0]) {
            // If the most significant bit are 0 then the integer is positive
            result = bitsToUInt(bits);
        } else {
            // If the most significant bit are 1 then the integer is negative
            // and the bits must be inverted and added 1 in order to get the
            // correct negative integer
            boolean[] b = new boolean[bits.length];
            for (int i = 0; i < bits.length; i++) {
                b[i] = !bits[i];
            }
            result = -1 * (bitsToUInt(b) + 1);
        }
        return result;
    }

    /**
     * copies an entire bit array into a new bit array
     *
     * @param b
     *            the bit array to copy
     */
    public static boolean[] copy(boolean[] b) {
        // Function just uses subset to copy
        return subset(b, 0, b.length);
    }


    /**
     * convert a byte (given as an integer) to bits with all bits turned
     *
     * @param in
     *            integer to convert, only the first byte are used
     */
    public static boolean[] rollByteToBits(int in) {
        boolean[] result = new boolean[8];
        for (int i = 7; i > -1; i--) {
            result[i] = (in % 2 == 1);
            in = in / 2;
        }
        return result;
    }

    /**
     * copies a subset from a bit array into a new bit array
     *
     * @param b
     *            bit array to copy from
     * @param start
     *            the index to start from
     * @param length
     *            the length of the subset
     *
     * @throws ArrayIndexOutOfBoundsException
     *             if subset exceeds the original arrays length (not
     *             declared)
     */
    public static boolean[] subset(boolean[] b, int start, int length) {
        boolean[] result;

        if (start >= b.length || start + length > b.length) {
            // Exception is thrown if the index starts before 0, or exceeds
            // the length of the original array
            throw new ArrayIndexOutOfBoundsException(
                    "Invalid subset: exceeds length of " + b.length
                            + ":\nstart of subset: " + start
                            + ", length of subset: " + length);
        } else {
            result = new boolean[length];
            System.arraycopy(b, start, result, 0, length);
        }
        return result;
    }
}

