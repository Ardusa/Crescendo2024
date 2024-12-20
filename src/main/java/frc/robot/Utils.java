package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Utils {
    public static class Vector3D {

        public double x;
        public double y;
        public double z;

        private static double sint, cost, newX, newY, newZ;

        public Vector3D(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Vector3D(Vector3D vector) {
            this(vector.x, vector.y, vector.z);
        }

        public Vector3D() {
            this(0, 0, 0);
        }

        public void rotateVectorAboutXAxis(double theta,
                Vector3D vectorToPutResultsIn) {
            sint = Math.sin(theta);
            cost = Math.cos(theta);

            newX = x;
            newY = y * cost - z * sint;
            newZ = y * sint + z * cost;
            // do calculations first, then apply. otherwise, if vectorToPutResultsIn
            // is also the invoking instance, it
            // causes problems
            vectorToPutResultsIn.x = newX;
            vectorToPutResultsIn.y = newY;
            vectorToPutResultsIn.z = newZ;
        }

        public void rotateVectorAboutYAxis(double theta,
                Vector3D vectorToPutResultsIn) {
            sint = Math.sin(theta);
            cost = Math.cos(theta);

            newX = z * sint + x * cost;
            newY = y;
            newZ = z * cost - x * sint;
            // do calculations first, then apply. otherwise, if vectorToPutResultsIn
            // is also the invoking instance, it
            // causes problems
            vectorToPutResultsIn.x = newX;
            vectorToPutResultsIn.y = newY;
            vectorToPutResultsIn.z = newZ;
        }

        /**
         * Calculates the rotation of this vector about the Z axis, then puts the
         * resulting coordinates into the vectorToPutResultsIn. Calling this method
         * never changes the vector it is called in.
         * 
         * @param theta
         *                             the angle specifying the how much to rotate
         * @param vectorToPutResultsIn
         *                             the vector which gets the coordinates resulting
         *                             from the
         *                             rotation calculation.
         */
        public void rotateVectorAboutZAxis(double theta,
                Vector3D vectorToPutResultsIn) {
            sint = Math.sin(theta);
            cost = Math.cos(theta);

            newX = x * cost - y * sint;
            newY = x * sint + y * cost;
            newZ = z;
            // do calculations first, then apply. otherwise, if vectorToPutResultsIn
            // is also the invoking instance, it
            // causes problems

            vectorToPutResultsIn.x = newX;
            vectorToPutResultsIn.y = newY;
            vectorToPutResultsIn.z = newZ;
        }

        /**
         * Adds the specified vector to this one and returns the result in a new
         * Vector.
         * 
         * @param b
         *          the Vector to subtract from this one
         */
        public Vector3D plus(Vector3D b) {
            Vector3D results = new Vector3D(0, 0, 0);
            this.plus(b, results);
            return results;
        }

        /**
         * Adds the specified vector to this one and returns the result in the
         * specified vectorToPutResultsIn
         * 
         * @param b
         *                             the Vector to add to this one
         * @param vectorToPutResultsIn
         *                             the Vector to put the results in
         */
        public void plus(Vector3D b, Vector3D vectorToPutResultsIn) {
            vectorToPutResultsIn.x = x + b.x;
            vectorToPutResultsIn.y = y + b.y;
            vectorToPutResultsIn.z = z + b.z;
        }

        /**
         * Subtracts the specified vector from this one and returns the result in a
         * new Vector.
         * 
         * @param b
         *          the Vector to subtract from this one
         * @return a new Vector with the result of the addition inside it.
         */
        public Vector3D minus(Vector3D b) {
            Vector3D results = new Vector3D(0, 0, 0);
            this.minus(b, results);
            return results;
        }

        /**
         * Subtracts the specified vector from this one and returns the result in
         * the specified vectorToPutResultsIn
         * 
         * @param b
         *                             the Vector to subtract from this one
         * @param vectorToPutResultsIn
         *                             the Vector to put the results in
         */
        public void minus(Vector3D b, Vector3D vectorToPutResultsIn) {
            vectorToPutResultsIn.x = x - b.x;
            vectorToPutResultsIn.y = y - b.y;
            vectorToPutResultsIn.z = z - b.z;
        }

        /**
         * Returns the cross product of the current vector (a) and the specified
         * vector (b). it returns aXb. The vector resulting from the cross product
         * has a direction which is the normal of a and b. The magnitude of the
         * cross product is the area of the parallelogram traced by the two vectors
         * a and b.
         */

        /**
         * Calculates the cross-product of this vector with the specified one.
         * 
         * @param b
         *          the Vector to cross with this one
         * @return a new Vector with the result of the cross product inside it.
         */
        public Vector3D cross(Vector3D b) {
            return cross(new Vector3D());
        }

        /**
         * Calculates the cross-product of this vector with the specified one. The
         * result is put into the specified vectorToPutResultsIn.
         * 
         * @param b
         *                             the Vector to cross with this one *
         * @param vectorToPutResultsIn
         *                             the Vector to put the results in
         */
        public void cross(Vector3D b, Vector3D vectorToPutResultsIn) {
            vectorToPutResultsIn.x = y * b.z - z * b.y;
            vectorToPutResultsIn.y = z * b.x - x * b.z;
            vectorToPutResultsIn.z = x * b.y - y * b.x;
        }

        /**
         * Performs a scalar multiplication on this vector and returns the result in
         * a new Vector.
         * 
         * @param scalarFactor
         *                     the scalar factor to multiply each of the vector's
         *                     components
         *                     by
         * @return a new Vector with the result of the scalar multiplication inside
         *         it.
         */
        public Vector3D times(double scalarFactor) {
            Vector3D results = new Vector3D();
            this.times(scalarFactor, results);
            return results;
        }

        /**
         * Performs a scalar multiplication on this vector and puts the result in a
         * the specified vectorToPutResultsIn.
         * 
         * @param scalarFactor
         *                     the scalar factor to multiply each of the vector's
         *                     components
         *                     by
         */
        public void times(double scalarFactor, Vector3D vectorToPutResultsIn) {
            vectorToPutResultsIn.x = x * scalarFactor;
            vectorToPutResultsIn.y = y * scalarFactor;
            vectorToPutResultsIn.z = z * scalarFactor;
        }

        /**
         * Calculates the normal from the specified 3 points. The cross product of
         * (a-b) and (c-b) is put into the vectorToPutNormalInto vector.
         * 
         * @param a
         *                              Vector a
         * @param b
         *                              Vector b, the vertex of the normal
         * @param c
         *                              Vector c
         * @param vectorToPutNormalInto
         *                              The vector which the normal calculation will be
         *                              put into. This
         *                              vector will be changed.
         */
        public void calculateNormal(Vector3D a, Vector3D b, Vector3D c,
                Vector3D vectorToPutNormalInto) {
            // B-b-C
            // |
            // a
            // |
            // A
            // a = A-B
            // b = C-B
            // normal = aXb
            double Ax = a.x - b.x;
            double Ay = a.y - b.y;
            double Az = a.z - b.z;

            double Bx = c.x - b.x;
            double By = c.y - b.y;
            double Bz = c.z - b.z;

            vectorToPutNormalInto.x = Ay * Bz - Az * By;
            vectorToPutNormalInto.y = Az * Bx - Ax * Bz;
            vectorToPutNormalInto.z = Ax * By - Ay * Bx;
        }

        /**
         * Calculates the distance between the two specified vectors.
         * 
         * @param a
         * @param b
         * @return the distance between vectors a and b
         */
        public double calculateDistance(Vector3D a, Vector3D b) {
            return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2)
                    + Math.pow(a.z - b.z, 2));
        }

        /**
         * Sets the values of this vector to the values from the specified vector.
         * 
         * @param vectorToCopy
         *                     the vector whose values will be copied into this one.
         */
        public void set(Vector3D vectorToCopy) {
            x = vectorToCopy.x;
            y = vectorToCopy.y;
            z = vectorToCopy.z;
        }
    }

    /**
     * X is Elbow
     * Y is Shoulder
     */
    public static class Vector2D {

        public double x;
        public double y;

        public Vector2D() {
        }

        /**
         * @param x Elbow
         * @param y Shoulder
         */
        public Vector2D(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Vector2D(Vector2D v) {
            set(v);
        }

        public Vector2D(Translation2d v) {
            this(v.getX(), v.getY());
        }

        public void set(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double getX() {
            return this.x;
        }

        public double getElbow() {
            return getX();
        }

        public double getY() {
            return this.y;
        }

        public double getShoulder() {
            return getY();
        }

        public void setX(double x) {
            this.x = x;
        }

        public void setElbow(double x) {
            setX(x);
        }

        public void setY(double y) {
            this.y = y;
        }

        public void setShoulder(double y) {
            setY(y);
        }

        public void set(Vector2D v) {
            this.x = v.x;
            this.y = v.y;
        }

        public void setZero() {
            x = 0;
            y = 0;
        }

        public double[] getComponents() {
            return new double[] { x, y };
        }

        public double getLength() {
            return Math.sqrt(x * x + y * y);
        }

        public double getLengthSq() {
            return (x * x + y * y);
        }

        /**
         * @param vx Elbow
         * @param vy Shoulder
         */
        public double distanceSq(double vx, double vy) {
            vx -= x;
            vy -= y;
            return (vx * vx + vy * vy);
        }

        public double distanceSq(Vector2D v) {
            double vx = v.x - this.x;
            double vy = v.y - this.y;
            return (vx * vx + vy * vy);
        }

        /**
         * @param vx Elbow
         * @param vy Shoulder
         */
        public double distance(double vx, double vy) {
            vx -= x;
            vy -= y;
            return Math.sqrt(vx * vx + vy * vy);
        }

        public double distance(Vector2D v) {
            double vx = v.x - this.x;
            double vy = v.y - this.y;
            return Math.sqrt(vx * vx + vy * vy);
        }

        public double getAngle() {
            return Math.atan2(y, x);
        }

        public void normalize() {
            double magnitude = getLength();
            x /= magnitude;
            y /= magnitude;
        }

        public Vector2D getNormalized() {
            double magnitude = getLength();
            return new Vector2D(x / magnitude, y / magnitude);
        }

        public void add(Vector2D v) {
            this.x += v.x;
            this.y += v.y;
        }

        /**
         * @param vx Elbow
         * @param vy Shoulder
         */
        public void add(double vx, double vy) {
            this.x += vx;
            this.y += vy;
        }

        public Vector2D getAdded(Vector2D v) {
            return new Vector2D(this.x + v.x, this.y + v.y);
        }

        public void subtract(Vector2D v) {
            this.x -= v.x;
            this.y -= v.y;
        }

        /**
         * @param vx Elbow
         * @param vy Shoulder
         */
        public void subtract(double vx, double vy) {
            this.x -= vx;
            this.y -= vy;
        }

        public Vector2D getSubtracted(Vector2D v) {
            return new Vector2D(this.x - v.x, this.y - v.y);
        }

        public void multiply(double scalar) {
            x *= scalar;
            y *= scalar;
        }

        public void elementwiseMultiply(double vx, double vy) {
            x *= vx;
            y *= vy;
        }

        public Vector2D getMultiplied(double scalar) {
            return new Vector2D(x * scalar, y * scalar);
        }

        public void divide(double scalar) {
            x /= scalar;
            y /= scalar;
        }

        public Vector2D getDivided(double scalar) {
            return new Vector2D(x / scalar, y / scalar);
        }

        public Vector2D getPerp() {
            return new Vector2D(-y, x);
        }

        public double dot(Vector2D v) {
            return (this.x * v.x + this.y * v.y);
        }

        /**
         * @param vx Elbow
         * @param vy Shoulder
         */
        public double dot(double vx, double vy) {
            return (this.x * vx + this.y * vy);
        }

        public double cross(Vector2D v) {
            return (this.x * v.y - this.y * v.x);
        }

        /**
         * @param vx Elbow
         * @param vy Shoulder
         */
        public double cross(double vx, double vy) {
            return (this.x * vy - this.y * vx);
        }

        public double project(Vector2D v) {
            return (this.dot(v) / this.getLength());
        }

        /**
         * @param vx Elbow
         * @param vy Shoulder
         */
        public double project(double vx, double vy) {
            return (this.dot(vx, vy) / this.getLength());
        }

        public Vector2D getProjectedVector(Vector2D v) {
            return this.getNormalized().getMultiplied(this.dot(v) / this.getLength());
        }

        public Vector2D getProjectedVector(double vx, double vy) {
            return this.getNormalized().getMultiplied(this.dot(vx, vy) / this.getLength());
        }

        public void exponentialFilter(Vector2D oldData, double alpha) {
            this.x = this.x * alpha + oldData.x * (1.0 - alpha);
            this.y = this.y * alpha + oldData.y * (1.0 - alpha);
        }

        public Vector2D getDerivative(Vector2D oldData, double sampleTime) {
            return new Vector2D((this.x - oldData.x) / sampleTime, (this.y - oldData.y) / sampleTime);
        }

        public void rotateBy(double angle) {
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            double rx = x * cos - y * sin;
            y = x * sin + y * cos;
            x = rx;
        }

        public Vector2D getRotatedBy(double angle) {
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            return new Vector2D(x * cos - y * sin, x * sin + y * cos);
        }

        public void reverse() {
            x = -x;
            y = -y;
        }

        public Vector2D getReversed() {
            return new Vector2D(-x, -y);
        }

        @Override
        public Vector2D clone() {
            return new Vector2D(x, y);
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == this) {
                return true;
            }
            if (obj instanceof Vector2D) {
                Vector2D v = (Vector2D) obj;
                return (x == v.x) && (y == v.y);
            }
            return false;
        }

        @Override
        public String toString() {
            return "Vector2d[" + x + ", " + y + "]";
        }

        public Translation2d toTranslation2d() {
            return new Translation2d(x, y);
        }
    }

    public static double degToRad(double x) {
        return (Math.PI / 180.0) * x;
    }

    public static double clampDegreeMeasurement(double a) {
        a = a % 360;
        if (a > 180) {
            a = a - 360;
        }
        if (a < -180) {
            a = a + 360;
        }

        return a;
    }

    public static boolean withinRange(Vector2D v, Vector2D current) {
        if (v.x >= current.x - Constants.kRange && v.x <= current.x + Constants.kRange) {
            if (v.y >= current.y - Constants.kRange && v.y <= current.y + Constants.kRange) {
                return true;
            }
        }
        return false;
    }

    public static boolean withinRange(Translation2d point1, Translation2d point2, Translation2d robotPose) {
        if (point1.getX() > point2.getX()) {
            if (point1.getY() > point2.getY()) {
                if (robotPose.getX() >= point2.getX() && robotPose.getX() <= point1.getX()) {
                    if (robotPose.getY() >= point2.getY() && robotPose.getY() <= point1.getY()) {
                        return true;
                    } else {
                        return false;
                    }
                } else {
                    return false;
                }
            } else {
                if (robotPose.getX() >= point2.getX() && robotPose.getX() <= point1.getX()) {
                    if (robotPose.getY() >= point1.getY() && robotPose.getY() <= point2.getY()) {
                        return true;
                    } else {
                        return false;
                    }
                } else {
                    return false;
                }
            }
        } else {
            if (point1.getY() > point2.getY()) {
                if (robotPose.getX() >= point1.getX() && robotPose.getX() <= point2.getX()) {
                    if (robotPose.getY() >= point2.getY() && robotPose.getY() <= point1.getY()) {
                        return true;
                    } else {
                        return false;
                    }
                } else {
                    return false;
                }
            } else {
                if (robotPose.getX() >= point1.getX() && robotPose.getX() <= point2.getX()) {
                    if (robotPose.getY() >= point1.getY() && robotPose.getY() <= point2.getY()) {
                        return true;
                    } else {
                        return false;
                    }
                } else {
                    return false;
                }
            }
        }
    }

    /**
     * Custom Deadzone Function that is exponential at the lower end and linear at
     * the upper end
     * 
     * @param input (double) Input to be deadzoned
     * @return (double) Deadzoned input
     */
    public static double customDeadzone(double input) {
        if (Math.abs(input) >= Constants.CustomDeadzone.kLowerLimitExpFunc
                && Math.abs(input) < Constants.CustomDeadzone.kUpperLimitExpFunc) {
            return Math.signum(input) * ((Constants.CustomDeadzone.kExpFuncMult)
                    * Math.pow((Constants.CustomDeadzone.kExpFuncBase), Math.abs(input))
                    - Constants.CustomDeadzone.kExpFuncConstant);
        } else if (Math.abs(input) >= Constants.CustomDeadzone.kUpperLimitExpFunc
                && Math.abs(input) <= Constants.CustomDeadzone.kUpperLimitLinFunc) {
            return Math.signum(input) * ((Constants.CustomDeadzone.kLinFuncMult)
                    * (Math.abs(input) - Constants.CustomDeadzone.kLinFuncOffset)
                    + (Constants.CustomDeadzone.kLinFuncConstant));
        }
        return Constants.CustomDeadzone.kNoSpeed;
    }

    public static boolean withinRange(double input, double range) {
        if (input >= -range && input <= range) {
            return true;
        }
        return false;
    }

    public static double pythagorean(double a, double b) {
        return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    }

    public static class LockHysteresis {
        private double mEngageError, mDisengageError;
        private boolean mLastState = false;

        public LockHysteresis(double engageError, double disengageError) {
            mEngageError = engageError;
            mDisengageError = disengageError;
        }

        public boolean calculate(double currentError) {
            currentError = Math.abs(currentError);
            if (mLastState == true) {
                if (currentError > mDisengageError) {
                    mLastState = false;
                }
            } else {

                if (currentError < mEngageError) {
                    mLastState = true;
                }
            }

            return mLastState;
        }

        public boolean get() {
            return mLastState;
        }

        public void reset() {
            mLastState = false;
        }
    }
}
