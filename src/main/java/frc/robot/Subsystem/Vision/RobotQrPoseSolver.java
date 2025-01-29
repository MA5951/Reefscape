package frc.robot.Subsystem.Vision;

public class RobotQrPoseSolver {
    // receives x, y pixel coordinates of the QR code corners
    public static double[] computeOffsetAndAngle(
            double topLeftX, double topLeftY,
            double topRightX, double topRightY,
            double bottomLeftX, double bottomLeftY,
            double bottomRightX, double bottomRightY) {

        
        double L = OffsetCalculatorConstants.QR_SIZE;
        double QR_Z = OffsetCalculatorConstants.QR_Z_OFFSET;
        double[][] worldPts = new double[][]{
                { -L / 2,  L / 2, QR_Z },  // Top-Left
                {  L / 2,  L / 2, QR_Z },  // Top-Right
                { -L / 2, -L / 2, QR_Z },  // Bottom-Left
                {  L / 2, -L / 2, QR_Z }   // Bottom-Right
        };

        
        double[][] imagePts = new double[][]{
                { topLeftX,      topLeftY },      // Top-Left
                { topRightX,     topRightY },     // Top-Right
                { bottomLeftX,   bottomLeftY },   // Bottom-Left
                { bottomRightX,  bottomRightY }   // Bottom-Right
        };


        double[][] H = computeHomography(worldPts, imagePts);

        
        double[][] K = {
                { OffsetCalculatorConstants.FX, 0.0, OffsetCalculatorConstants.CX },
                { 0.0, OffsetCalculatorConstants.FY, OffsetCalculatorConstants.CY },
                { 0.0, 0.0, 1.0 }
        };
        double[][] Kinv = invert3x3(K);

        double[][] B = multiply3x3And3x3(Kinv, H); // B = K^-1 * H

        
        double[] b1 = { B[0][0], B[1][0], B[2][0] };
        double[] b2 = { B[0][1], B[1][1], B[2][1] };
        double[] b3 = { B[0][2], B[1][2], B[2][2] };
       
        double scale = 1.0 / norm(b1);
    
        double[] r1 = multiplyVecScalar(b1, scale);
        double[] r2 = multiplyVecScalar(b2, scale);
        double[] tCam = multiplyVecScalar(b3, scale);
        
        double[] r3 = cross(r1, r2);

        double[][] R_qr_to_cam = {
                { r1[0], r2[0], r3[0] },
                { r1[1], r2[1], r3[1] },
                { r1[2], r2[2], r3[2] }
        };

        
        double[][] R_cam_to_robot = rotationFromRollPitchYawDeg(
                OffsetCalculatorConstants.CAM_ROLL_DEG,
                OffsetCalculatorConstants.CAM_PITCH_DEG,
                OffsetCalculatorConstants.CAM_YAW_DEG
        );

       
        double[] camOffsetRobot = { OffsetCalculatorConstants.CAM_X_OFFSET,
                                     OffsetCalculatorConstants.CAM_Y_OFFSET,
                                     OffsetCalculatorConstants.CAM_Z_OFFSET };

        

        double[] pQrInRobot = addVec(multiply3x3AndVec(R_cam_to_robot, tCam), camOffsetRobot);

        double[][] R_qr_to_robot = multiply3x3And3x3(R_cam_to_robot, R_qr_to_cam);

        
        double[][] R_robot_to_qr = transpose3x3(R_qr_to_robot);

        
        double[] pRobotInQr = multiply3x3AndVec(R_robot_to_qr, pQrInRobot);
        pRobotInQr = multiplyVecScalar(pRobotInQr, -1.0);

        double xOffset = pRobotInQr[0];
        double yOffset = pRobotInQr[1];
        double zRobot  = pRobotInQr[2]; // Not used for 2D navigation

        
        double heading = Math.atan2(R_robot_to_qr[1][0], R_robot_to_qr[0][0]);


        return new double[]{ xOffset, yOffset, heading };
    }

    // Helper Methods for Matrix Operations
   
    private static double[][] computeHomography(double[][] worldPts, double[][] imagePts) {
        double[][] A = new double[8][8];
        double[] b = new double[8];

        for (int i = 0; i < 4; i++) {
            double X = worldPts[i][0];
            double Y = worldPts[i][1];
            double u = imagePts[i][0];
            double v = imagePts[i][1];

            A[2 * i][0] = X;
            A[2 * i][1] = Y;
            A[2 * i][2] = 1;
            A[2 * i][3] = 0;
            A[2 * i][4] = 0;
            A[2 * i][5] = 0;
            A[2 * i][6] = -X * u;
            A[2 * i][7] = -Y * u;
            b[2 * i] = u;

            A[2 * i + 1][0] = 0;
            A[2 * i + 1][1] = 0;
            A[2 * i + 1][2] = 0;
            A[2 * i + 1][3] = X;
            A[2 * i + 1][4] = Y;
            A[2 * i + 1][5] = 1;
            A[2 * i + 1][6] = -X * v;
            A[2 * i + 1][7] = -Y * v;
            b[2 * i + 1] = v;
        }

        double[] h = solveLinearSystem(A, b);

        double[][] H = {
                { h[0], h[1], h[2] },
                { h[3], h[4], h[5] },
                { h[6], h[7], 1.0 }
        };

        return H;
    }

    private static double[] solveLinearSystem(double[][] A, double[] b) {
        int n = A.length;
        double[][] M = new double[n][n + 1];

        for (int i = 0; i < n; i++) {
            System.arraycopy(A[i], 0, M[i], 0, n);
            M[i][n] = b[i];
        }

        for (int i = 0; i < n; i++) {
            int maxRow = i;
            double maxVal = Math.abs(M[i][i]);
            for (int k = i + 1; k < n; k++) {
                if (Math.abs(M[k][i]) > maxVal) {
                    maxVal = Math.abs(M[k][i]);
                    maxRow = k;
                }
            }

            double[] temp = M[i];
            M[i] = M[maxRow];
            M[maxRow] = temp;

            if (Math.abs(M[i][i]) < 1e-12) {
                throw new ArithmeticException("Matrix is singular or nearly singular.");
            }

            for (int j = i; j <= n; j++) {
                M[i][j] /= M[i][i];
            }

            for (int k = i + 1; k < n; k++) {
                double factor = M[k][i];
                for (int j = i; j <= n; j++) {
                    M[k][j] -= factor * M[i][j];
                }
            }
        }

        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            x[i] = M[i][n];
            for (int j = i + 1; j < n; j++) {
                x[i] -= M[i][j] * x[j];
            }
        }

        return x;
    }

    private static double[][] invert3x3(double[][] mat) {
        double a = mat[0][0], b = mat[0][1], c = mat[0][2];
        double d = mat[1][0], e = mat[1][1], f = mat[1][2];
        double g = mat[2][0], h = mat[2][1], i = mat[2][2];

        double A = e * i - f * h;
        double B = -(d * i - f * g);
        double C = d * h - e * g;
        double D = -(b * i - c * h);
        double E = a * i - c * g;
        double F = -(a * h - b * g);
        double G = b * f - c * e;
        double H = -(a * f - c * d);
        double I = a * e - b * d;

        double det = a * A + b * B + c * C;
        if (Math.abs(det) < 1e-12) {
            throw new ArithmeticException("Matrix is singular and cannot be inverted.");
        }
        double invDet = 1.0 / det;

        double[][] inv = {
                { A * invDet, D * invDet, G * invDet },
                { B * invDet, E * invDet, H * invDet },
                { C * invDet, F * invDet, I * invDet }
        };

        return inv;
    }

    private static double[][] multiply3x3And3x3(double[][] A, double[][] B) {
        double[][] C = new double[3][3];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                C[r][c] = 0.0;
                for (int k = 0; k < 3; k++) {
                    C[r][c] += A[r][k] * B[k][c];
                }
            }
        }
        return C;
    }

    private static double[] multiply3x3AndVec(double[][] M, double[] v) {
        double[] result = new double[3];
        for (int i = 0; i < 3; i++) {
            result[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
        }
        return result;
    }

    private static double[][] transpose3x3(double[][] M) {
        double[][] T = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T[i][j] = M[j][i];
            }
        }
        return T;
    }

    private static double[][] rotationFromRollPitchYawDeg(double rollDeg, double pitchDeg, double yawDeg) {
        double roll = Math.toRadians(rollDeg);
        double pitch = Math.toRadians(pitchDeg);
        double yaw = Math.toRadians(yawDeg);

        // Rotation matrices around X, Y, Z axes
        double[][] Rx = {
                { 1, 0, 0 },
                { 0, Math.cos(roll), -Math.sin(roll) },
                { 0, Math.sin(roll),  Math.cos(roll) }
        };

        double[][] Ry = {
                { Math.cos(pitch), 0, Math.sin(pitch) },
                { 0, 1, 0 },
                { -Math.sin(pitch), 0, Math.cos(pitch) }
        };

        double[][] Rz = {
                { Math.cos(yaw), -Math.sin(yaw), 0 },
                { Math.sin(yaw),  Math.cos(yaw), 0 },
                { 0, 0, 1 }
        };

        double[][] Rzy = multiply3x3And3x3(Rz, Ry);
        double[][] R = multiply3x3And3x3(Rzy, Rx);

        return R;
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
        };
    }

    private static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double norm(double[] v) {
        return Math.sqrt(dot(v, v));
    }

    private static double[] multiplyVecScalar(double[] v, double s) {
        return new double[]{ v[0] * s, v[1] * s, v[2] * s };
    }

    private static double[] addVec(double[] a, double[] b) {
        return new double[]{ a[0] + b[0], a[1] + b[1], a[2] + b[2] };
    }
}
