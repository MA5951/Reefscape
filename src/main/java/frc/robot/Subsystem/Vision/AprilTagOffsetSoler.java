// package frc.robot.Subsystem.Vision;

// public class AprilTagOffsetSolver {
//     /**
//      * Receives x, y pixel coordinates of the QR code corners and computes the robot's
//      * 2D offset (x, y) and heading angle theta relative to the QR code's center and orientation.
//      *
//      * @param topLeftX      Pixel X coordinate of the top-left corner of the QR code
//      * @param topLeftY      Pixel Y coordinate of the top-left corner
//      * @param topRightX     Pixel X coordinate of the top-right corner
//      * @param topRightY     Pixel Y coordinate of the top-right corner
//      * @param bottomLeftX   Pixel X coordinate of the bottom-left corner
//      * @param bottomLeftY   Pixel Y coordinate of the bottom-left corner
//      * @param bottomRightX  Pixel X coordinate of the bottom-right corner
//      * @param bottomRightY  Pixel Y coordinate of the bottom-right corner
//      * @return A double array containing [xOffset, yOffset, headingTheta]
//      *         where xOffset and yOffset are in meters, and headingTheta is in radians
//      */
//     public static double[] computeOffsetAndAngle(
//             double topLeftX, double topLeftY,
//             double topRightX, double topRightY,
//             double bottomLeftX, double bottomLeftY,
//             double bottomRightX, double bottomRightY) {

//         // Define real-world coordinates of QR code corners (World Frame)
//         double L = AprilTagOffsetCalculatorConstants.QR_SIZE;
//         double QR_Z = AprilTagOffsetCalculatorConstants.QR_Z_OFFSET;
//         double[][] worldPts = new double[][]{
//                 { -L / 2,  L / 2, QR_Z },  // Top-Left
//                 {  L / 2,  L / 2, QR_Z },  // Top-Right
//                 { -L / 2, -L / 2, QR_Z },  // Bottom-Left
//                 {  L / 2, -L / 2, QR_Z }   // Bottom-Right
//         };

//         // Define image coordinates of QR code corners (Image Frame)
//         double[][] imagePts = new double[][]{
//                 { topLeftX,      topLeftY },      // Top-Left
//                 { topRightX,     topRightY },     // Top-Right
//                 { bottomLeftX,   bottomLeftY },   // Bottom-Left
//                 { bottomRightX,  bottomRightY }   // Bottom-Right
//         };

//         // Compute Homography Matrix H that maps worldPts to imagePts
//         double[][] H = computeHomography(worldPts, imagePts);

//         // Camera Intrinsic Matrix K
//         double[][] K = {
//                 { AprilTagOffsetCalculatorConstants.FX, 0.0, AprilTagOffsetCalculatorConstants.CX },
//                 { 0.0, AprilTagOffsetCalculatorConstants.FY, AprilTagOffsetCalculatorConstants.CY },
//                 { 0.0, 0.0, 1.0 }
//         };
//         double[][] Kinv = invert3x3(K);

//         // Compute B = K^-1 * H
//         double[][] B = multiply3x3And3x3(Kinv, H);

//         // Extract columns from B
//         double[] b1 = { B[0][0], B[1][0], B[2][0] };
//         double[] b2 = { B[0][1], B[1][1], B[2][1] };
//         double[] b3 = { B[0][2], B[1][2], B[2][2] };

//         // Compute scale factor
//         double scale = 1.0 / norm(b1);

//         // Compute rotation columns and translation vector
//         double[] r1 = multiplyVecScalar(b1, scale);
//         double[] r2 = multiplyVecScalar(b2, scale);
//         double[] tCam = multiplyVecScalar(b3, scale);

//         // Compute r3 as cross product of r1 and r2 to ensure orthogonality
//         double[] r3 = cross(r1, r2);

//         // Assemble rotation matrix R_qr_to_cam
//         double[][] R_qr_to_cam = {
//                 { r1[0], r2[0], r3[0] },
//                 { r1[1], r2[1], r3[1] },
//                 { r1[2], r2[2], r3[2] }
//         };

//         // Compute rotation matrix from camera to robot frame based on camera orientation angles
//         double[][] R_cam_to_robot = rotationFromRollPitchYawDeg(
//                 AprilTagOffsetCalculatorConstants.CAM_ROLL_DEG,
//                 AprilTagOffsetCalculatorConstants.CAM_PITCH_DEG,
//                 AprilTagOffsetCalculatorConstants.CAM_YAW_DEG
//         );

//         // Camera offset relative to robot center
//         double[] camOffsetRobot = { AprilTagOffsetCalculatorConstants.CAM_X_OFFSET,
//                                      AprilTagOffsetCalculatorConstants.CAM_Y_OFFSET,
//                                      AprilTagOffsetCalculatorConstants.CAM_Z_OFFSET };

//         // Position of QR origin in robot frame:
//         // p_qr_in_robot = R_cam_to_robot * tCam + camOffsetRobot
//         double[] pQrInRobot = addVec(multiply3x3AndVec(R_cam_to_robot, tCam), camOffsetRobot);

//         // Rotation from QR frame to robot frame
//         double[][] R_qr_to_robot = multiply3x3And3x3(R_cam_to_robot, R_qr_to_cam);

//         // Invert transformation to get robot's pose in QR frame
//         double[][] R_robot_to_qr = transpose3x3(R_qr_to_robot(R_qr_to_robot));

//         // Compute robot's position in QR frame
//         double[] pRobotInQr = multiply3x3AndVec(R_robot_to_qr, pQrInRobot);
//         pRobotInQr = multiplyVecScalar(pRobotInQr, -1.0);

//         // Extract 2D offset and heading angle
//         double xOffset = pRobotInQr[0];
//         double yOffset = pRobotInQr[1];
//         double zRobot  = pRobotInQr[2]; // Not used for 2D navigation

//         double heading = Math.atan2(R_robot_to_qr[1][0], R_robot_to_qr[0][0]);

//         return new double[]{ xOffset, yOffset, heading };
//     }

//     //-------------------------------------------------------------------------
//     // Helper Methods for Matrix Operations
//     //-------------------------------------------------------------------------

//     /**
//      * Computes the homography matrix H that maps worldPts to imagePts.
//      * Assumes four point correspondences.
//      *
//      * @param worldPts Array of four [X, Y, Z] world points (all lie on the QR plane)
//      * @param imagePts Array of four [u, v] image points
//      * @return 3x3 Homography matrix H
//      */
//     private static double[][] computeHomography(double[][] worldPts, double[][] imagePts) {
//         // We have four correspondences, leading to eight equations
//         // h11*X + h12*Y + h13 - h31*X*u - h32*Y*u = u
//         // h21*X + h22*Y + h23 - h31*X*v - h32*Y*v = v
//         //
//         // Form the system A * h = b, where h is [h11 h12 h13 h21 h22 h23 h31 h32]^T
//         double[][] A = new double[8][8];
//         double[] b = new double[8];

//         for (int i = 0; i < 4; i++) {
//             double X = worldPts[i][0];
//             double Y = worldPts[i][1];
//             double u = imagePts[i][0];
//             double v = imagePts[i][1];

//             // First equation for each point
//             A[2 * i][0] = X;
//             A[2 * i][1] = Y;
//             A[2 * i][2] = 1;
//             A[2 * i][3] = 0;
//             A[2 * i][4] = 0;
//             A[2 * i][5] = 0;
//             A[2 * i][6] = -X * u;
//             A[2 * i][7] = -Y * u;
//             b[2 * i] = u;

//             // Second equation for each point
//             A[2 * i + 1][0] = 0;
//             A[2 * i + 1][1] = 0;
//             A[2 * i + 1][2] = 0;
//             A[2 * i + 1][3] = X;
//             A[2 * i + 1][4] = Y;
//             A[2 * i + 1][5] = 1;
//             A[2 * i + 1][6] = -X * v;
//             A[2 * i + 1][7] = -Y * v;
//             b[2 * i + 1] = v;
//         }

//         // Solve A * h = b
//         double[] h = solveLinearSystem(A, b);

//         // Assemble Homography matrix H
//         double[][] H = {
//                 { h[0], h[1], h[2] },
//                 { h[3], h[4], h[5] },
//                 { h[6], h[7], 1.0 }
//         };

//         return H;
//     }

//     /**
//      * Solves a linear system A * x = b using Gaussian elimination with partial pivoting.
//      * Assumes A is a square matrix and has a unique solution.
//      *
//      * @param A Coefficient matrix (n x n)
//      * @param b Right-hand side vector (n)
//      * @return Solution vector x (n)
//      */
//     private static double[] solveLinearSystem(double[][] A, double[] b) {
//         int n = A.length;
//         double[][] M = new double[n][n + 1];

//         // Build augmented matrix
//         for (int i = 0; i < n; i++) {
//             System.arraycopy(A[i], 0, M[i], 0, n);
//             M[i][n] = b[i];
//         }

//         // Gaussian elimination with partial pivoting
//         for (int i = 0; i < n; i++) {
//             // Partial pivoting: find the maximum element in the current column
//             int maxRow = i;
//             double maxVal = Math.abs(M[i][i]);
//             for (int k = i + 1; k < n; k++) {
//                 if (Math.abs(M[k][i]) > maxVal) {
//                     maxVal = Math.abs(M[k][i]);
//                     maxRow = k;
//                 }
//             }

//             // Swap maximum row with current row (pivot)
//             double[] temp = M[i];
//             M[i] = M[maxRow];
//             M[maxRow] = temp;

//             // Check for singular matrix
//             if (Math.abs(M[i][i]) < 1e-12) {
//                 throw new ArithmeticException("Matrix is singular or nearly singular.");
//             }

//             // Normalize pivot row
//             for (int j = i; j <= n; j++) {
//                 M[i][j] /= M[i][i];
//             }

//             // Eliminate all rows below the pivot
//             for (int k = i + 1; k < n; k++) {
//                 double factor = M[k][i];
//                 for (int j = i; j <= n; j++) {
//                     M[k][j] -= factor * M[i][j];
//                 }
//             }
//         }

//         // Back substitution
//         double[] x = new double[n];
//         for (int i = n - 1; i >= 0; i--) {
//             x[i] = M[i][n];
//             for (int j = i + 1; j < n; j++) {
//                 x[i] -= M[i][j] * x[j];
//             }
//             // No need to divide by M[i][i] since it's already normalized
//         }

//         return x;
//     }

//     /**
//      * Inverts a 3x3 matrix using the adjugate method.
//      *
//      * @param mat 3x3 matrix to invert
//      * @return Inverted 3x3 matrix
//      */
//     private static double[][] invert3x3(double[][] mat) {
//         double a = mat[0][0], b = mat[0][1], c = mat[0][2];
//         double d = mat[1][0], e = mat[1][1], f = mat[1][2];
//         double g = mat[2][0], h = mat[2][1], i = mat[2][2];

//         double A = e * i - f * h;
//         double B = -(d * i - f * g);
//         double C = d * h - e * g;
//         double D = -(b * i - c * h);
//         double E = a * i - c * g;
//         double F = -(a * h - b * g);
//         double G = b * f - c * e;
//         double H = -(a * f - c * d);
//         double I = a * e - b * d;

//         double det = a * A + b * B + c * C;
//         if (Math.abs(det) < 1e-12) {
//             throw new ArithmeticException("Matrix is singular and cannot be inverted.");
//         }
//         double invDet = 1.0 / det;

//         double[][] inv = {
//                 { A * invDet, D * invDet, G * invDet },
//                 { B * invDet, E * invDet, H * invDet },
//                 { C * invDet, F * invDet, I * invDet }
//         };

//         return inv;
//     }

//     /**
//      * Multiplies two 3x3 matrices.
//      *
//      * @param A First 3x3 matrix
//      * @param B Second 3x3 matrix
//      * @return Product of A and B (3x3 matrix)
//      */
//     private static double[][] multiply3x3And3x3(double[][] A, double[][] B) {
//         double[][] C = new double[3][3];
//         for (int r = 0; r < 3; r++) {
//             for (int c = 0; c < 3; c++) {
//                 C[r][c] = 0.0;
//                 for (int k = 0; k < 3; k++) {
//                     C[r][c] += A[r][k] * B[k][c];
//                 }
//             }
//         }
//         return C;
//     }

//     /**
//      * Multiplies a 3x3 matrix with a 3-element vector.
//      *
//      * @param M 3x3 matrix
//      * @param v 3-element vector
//      * @return Resulting 3-element vector
//      */
//     private static double[] multiply3x3AndVec(double[][] M, double[] v) {
//         double[] result = new double[3];
//         for (int i = 0; i < 3; i++) {
//             result[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
//         }
//         return result;
//     }

//     /**
//      * Transposes a 3x3 matrix.
//      *
//      * @param M 3x3 matrix
//      * @return Transposed 3x3 matrix
//      */
//     private static double[][] transpose3x3(double[][] M) {
//         double[][] T = new double[3][3];
//         for (int i = 0; i < 3; i++) {
//             for (int j = 0; j < 3; j++) {
//                 T[i][j] = M[j][i];
//             }
//         }
//         return T;
//     }

//     /**
//      * Computes the rotation matrix from roll, pitch, and yaw angles in degrees.
//      * Assumes intrinsic Tait-Bryan angles: Rz(yaw) * Ry(pitch) * Rx(roll).
//      *
//      * @param rollDeg  Roll angle in degrees
//      * @param pitchDeg Pitch angle in degrees
//      * @param yawDeg   Yaw angle in degrees
//      * @return 3x3 rotation matrix
//      */
//     private static double[][] rotationFromRollPitchYawDeg(double rollDeg, double pitchDeg, double yawDeg) {
//         double roll = Math.toRadians(rollDeg);
//         double pitch = Math.toRadians(pitchDeg);
//         double yaw = Math.toRadians(yawDeg);

//         // Rotation matrices around X, Y, Z axes
//         double[][] Rx = {
//                 { 1, 0, 0 },
//                 { 0, Math.cos(roll), -Math.sin(roll) },
//                 { 0, Math.sin(roll),  Math.cos(roll) }
//         };

//         double[][] Ry = {
//                 { Math.cos(pitch), 0, Math.sin(pitch) },
//                 { 0, 1, 0 },
//                 { -Math.sin(pitch), 0, Math.cos(pitch) }
//         };

//         double[][] Rz = {
//                 { Math.cos(yaw), -Math.sin(yaw), 0 },
//                 { Math.sin(yaw),  Math.cos(yaw), 0 },
//                 { 0, 0, 1 }
//         };

//         // Combined rotation: Rz * Ry * Rx
//         double[][] Rzy = multiply3x3And3x3(Rz, Ry);
//         double[][] R = multiply3x3And3x3(Rzy, Rx);

//         return R;
//     }

//     /**
//      * Computes the cross product of two 3-element vectors.
//      *
//      * @param a First vector
//      * @param b Second vector
//      * @return Cross product vector
//      */
//     private static double[] cross(double[] a, double[] b) {
//         return new double[]{
//                 a[1] * b[2] - a[2] * b[1],
//                 a[2] * b[0] - a[0] * b[2],
//                 a[0] * b[1] - a[1] * b[0]
//         };
//     }

//     /**
//      * Computes the dot product of two 3-element vectors.
//      *
//      * @param a First vector
//      * @param b Second vector
//      * @return Dot product
//      */
//     private static double dot(double[] a, double[] b) {
//         return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
//     }

//     /**
//      * Computes the Euclidean norm of a 3-element vector.
//      *
//      * @param v Vector
//      * @return Euclidean norm
//      */
//     private static double norm(double[] v) {
//         return Math.sqrt(dot(v, v));
//     }

//     /**
//      * Multiplies a vector by a scalar.
//      *
//      * @param v Vector
//      * @param s Scalar
//      * @return Scaled vector
//      */
//     private static double[] multiplyVecScalar(double[] v, double s) {
//         return new double[]{ v[0] * s, v[1] * s, v[2] * s };
//     }

//     /**
//      * Adds two 3-element vectors.
//      *
//      * @param a First vector
//      * @param b Second vector
//      * @return Sum of vectors
//      */
//     private static double[] addVec(double[] a, double[] b) {
//         return new double[]{ a[0] + b[0], a[1] + b[1], a[2] + b[2] };
//     }

//     /**
//      * Generates the transpose of the given rotation matrix.
//      *
//      * @param R_qr_to_robot Rotation matrix from QR to Robot frame
//      * @return Transposed rotation matrix (Robot to QR frame)
//      */
//     private static double[][] R_robot_to_qr(double[][] R_qr_to_robot) {
//         return transpose3x3(R_qr_to_robot);
//     }
// }
