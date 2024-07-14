package com.example.prototypedgps;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;

public class TransformerCoordinate {

    public static double[] CHTRS95toMN95hBessel(RealMatrix X) {

        // Transformation CHTRS95 => CH1903+
        RealMatrix t = new Array2DRowRealMatrix(new double[][]{
                {-674.374},
                {-15.056},
                {-405.346}
        });
        t = X.add(t);

        // Param ellipsoid Bessel
        double[] ellParam = ellBesselParam();
        double e = ellParam[0];

        // Ellipsoidal coordinates CH1903+
        double[] result = cart2ell(Constants.ELL_A_Bessel, e, t);


        return ell2EN(result[0], result[1], result[2]);
    }

    public static double[] MN95toCHTRS(double east, double north, double ellHeight) {

        double[] ellCH1903p = EN2ell(east,north);
        double[] X_CH1903p = ell2cart(ellCH1903p[0],ellCH1903p[1],ellHeight);
        double x_CHTRS = X_CH1903p[0] + 674.374;
        double y_CHTRS = X_CH1903p[1] + 15.056;
        double z_CHTRS = X_CH1903p[2] + 405.346;

        return new double[]{x_CHTRS,y_CHTRS,z_CHTRS};
    }

    public static double[] cart2ell(double a, double e, RealMatrix t) {
        double[] result = new double[3];
        double x = t.getEntry(0, 0);
        double y = t.getEntry(1, 0);
        double z = t.getEntry(2, 0);

        // Ellipsoidal longitude radians
        double lon_rad = Math.atan2(y, x);

        // Ellipsoidal longitude degrees
        double lon_deg = lon_rad * 180.0 / Math.PI;

        // Initialisation ellipsoidal height and radius of curvature of the normal section
        double hi = 0;
        double him1 = 1;
        double RNi = 1;
        double lat_radi = 0;

        // Iterative loop to calculate latitude and ellipsoidal height
        while (Math.abs(hi - him1) > 0.000001) {
            him1 = hi;

            double sqrt = Math.sqrt(x * x + y * y);
            lat_radi = Math.atan2(z, sqrt * (1 - (RNi / (RNi + hi)) * e * e));
            RNi = a / Math.sqrt(1 - e * e * Math.sin(lat_radi) * Math.sin(lat_radi));
            hi = sqrt / Math.cos(lat_radi) - RNi;
        }

        // Latitude in degrees and ellipsoidal height
        double lat_deg = lat_radi * 180.0 / Math.PI;
        double h = hi;

        result[0] = lon_deg;
        result[1] = lat_deg;
        result[2] = h;

        return result;
    }

    public static double[] ell2cart(double lon_deg, double lat_deg, double h) {
        double lon_rad = Math.toRadians(lon_deg);
        double lat_rad = Math.toRadians(lat_deg);

        double[] paramBessel = ellBesselParam();
        double RN = Constants.ELL_A_Bessel/Math.sqrt(1.0-Math.pow(paramBessel[0],2.0)*
                Math.pow(Math.sin(lat_rad),2.0));

        double x = (RN + h) * Math.cos(lat_rad) * Math.cos(lon_rad);
        double y = (RN + h) * Math.cos(lat_rad) * Math.sin(lon_rad);
        double z = (RN * (1 - paramBessel[0] * paramBessel[0]) + h) * Math.sin(lat_rad);
        return new double[]{x,y,z};
    }

    public static double[] ell2EN(double lon_deg, double lat_deg, double height) {

        double lon = lon_deg * Math.PI / 180.0;
        double lat = lat_deg * Math.PI / 180.0;

        double lon0 = (7.0 + 26.0 / 60.0 + 22.50 / 3600.0) * Math.PI / 180.0;
        double alpha = 1.0007291384304;
        double k = 1.0030714396280;
        double lat_sph_0 = (46.0 + 54.0 / 60.0 + 27.83324846 / 3600.0) * Math.PI / 180.0;
        double R_sph = 6378815.90365;
        double E0, N0;


        E0 = 2600000.000;
        N0 = 1200000.000;


        // ellipsoid -> normal sphere
        double lon_sph = alpha * (lon - lon0);
        double lat_sph = 2 * Math.atan(
                k * Math.pow(Math.tan(Math.PI / 4 + lat / 2.0), alpha) * Math.pow((1 - ellBesselParam()[0] * Math.sin(lat)) / (1 + ellBesselParam()[0] * Math.sin(lat)),
                        alpha * ellBesselParam()[0] / 2.0))
                - Math.PI / 2.0;

        // normal sphere -> oblique sphere
        double lon_sph_t = Math.atan(Math.sin(lon_sph) / (Math.sin(lat_sph_0) * Math.tan(lat_sph)
                + Math.cos(lat_sph_0) * Math.cos(lon_sph)));
        double lat_sph_t = Math.asin(Math.cos(lat_sph_0) * Math.sin(lat_sph)
                - Math.sin(lat_sph_0) * Math.cos(lat_sph) * Math.cos(lon_sph));

        // oblique sphere -> plane
        double E = E0 + R_sph * lon_sph_t;
        double N = N0 + R_sph * Math.log(Math.tan(Math.PI / 4.0 + lat_sph_t / 2.0));

        return new double[]{E,N,height};
    }

    public static double[] EN2ell(double east, double north) {

        double[] besselParam = ellBesselParam();

        double lon0 = (7.0 + 26.0 / 60.0 + 22.50 / 3600.0) * Math.PI / 180.0;
        double alpha = 1.0007291384304;
        double k = 1.0030714396280;
        double lat_sph_0 = (46.0 + 54.0 / 60.0 + 27.83324846 / 3600.0) * Math.PI / 180.0;
        double R_sph = 6378815.90365;

        double east0 = 2600000.000;
        double north0 = 1200000.000;

        double lon_sph_t = (east-east0)/R_sph;
        double lat_sph_t = 2.0*Math.atan(Math.exp((north-north0)/R_sph))-Math.PI/2.0;

        double lon_sph = Math.atan(Math.sin(lon_sph_t)/(Math.cos(lat_sph_0)*Math.cos(lon_sph_t) - Math.sin(lat_sph_0)*Math.tan(lat_sph_t)));
        double lat_sph = Math.asin(Math.cos(lat_sph_0)*Math.sin(lat_sph_t)+ Math.sin(lat_sph_0)*Math.cos(lat_sph_t)*Math.cos(lon_sph_t));

        double lon = lon0 + 1.0/alpha*lon_sph;
        double lati = lat_sph+1.0;
        double lat = lat_sph;
        while(Math.abs(lati-lat)>0.000000000001){
            lati=lat;
            lat = 2.0*Math.atan(
                    Math.pow(Math.tan(Math.PI/4.0+lat_sph/2.0),1.0/alpha) *
                            Math.pow(k, -1.0/alpha) *
                            Math.pow((1.0+besselParam[0]*Math.sin(lati))/
                                            (1.0-besselParam[0]*Math.sin(lati)),
                                    besselParam[0]/2.0))
                    - Math.PI/2.0;
        }
        double lon_deg = Math.toDegrees(lon);
        double lat_deg = Math.toDegrees(lat);

        return new double[]{lon_deg,lat_deg};
    }

    public static double[] ellBesselParam() {
        double a = Constants.ELL_A_Bessel;
        double f = Constants.ELL_F_Bessel;
        double b = a - a * f;
        double e = Math.sqrt(a * a - b * b) / a;
        return new double[]{e, b};
    }
}
