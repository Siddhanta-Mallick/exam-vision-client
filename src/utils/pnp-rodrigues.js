// Pure JavaScript implementation of cv.solvePnP and cv.Rodrigues

/**
 * Solves PnP (Perspective-n-Point) problem using Levenberg-Marquardt optimization
 * @param {Array} objectPoints - 3D points in world coordinate [x, y, z, ...]
 * @param {Array} imagePoints - 2D points in image coordinate [u, v, ...]
 * @param {Array} cameraMatrix - 3x3 camera intrinsic matrix [fx, 0, cx, 0, fy, cy, 0, 0, 1]
 * @returns {Object} {rvec: [rx, ry, rz], tvec: [tx, ty, tz]}
 */
export function solvePnP(objectPoints, imagePoints, cameraMatrix) {
    const numPoints = objectPoints.length / 3;
    
    // Extract camera parameters
    const fx = cameraMatrix[0];
    const fy = cameraMatrix[4];
    const cx = cameraMatrix[2];
    const cy = cameraMatrix[5];
    
    // Initial guess using DLT (Direct Linear Transform)
    let pose = initializePose(objectPoints, imagePoints, fx, fy, cx, cy, numPoints);
    
    // Refine using Levenberg-Marquardt
    pose = refinePose(pose, objectPoints, imagePoints, fx, fy, cx, cy, numPoints);
    
    return pose;
}

/**
 * Converts rotation vector to rotation matrix (Rodrigues formula)
 * @param {Array} rvec - Rotation vector [rx, ry, rz]
 * @returns {Array} 3x3 rotation matrix in row-major order
 */
export function rodrigues(rvec) {
    const [rx, ry, rz] = rvec;
    const theta = Math.sqrt(rx * rx + ry * ry + rz * rz);
    
    if (theta < 1e-10) {
        return [1, 0, 0, 0, 1, 0, 0, 0, 1];
    }
    
    const [kx, ky, kz] = [rx / theta, ry / theta, rz / theta];
    const c = Math.cos(theta);
    const s = Math.sin(theta);
    const t = 1 - c;
    
    return [
        c + kx * kx * t,      kx * ky * t - kz * s, kx * kz * t + ky * s,
        ky * kx * t + kz * s, c + ky * ky * t,      ky * kz * t - kx * s,
        kz * kx * t - ky * s, kz * ky * t + kx * s, c + kz * kz * t
    ];
}

// Helper: Initialize pose using simplified method
function initializePose(objPts, imgPts, fx, fy, cx, cy, n) {
    // Use POSIT-like algorithm for better initialization
    let cx3d = 0, cy3d = 0, cz3d = 0;
    let cx2d = 0, cy2d = 0;
    
    for (let i = 0; i < n; i++) {
        cx3d += objPts[i * 3];
        cy3d += objPts[i * 3 + 1];
        cz3d += objPts[i * 3 + 2];
        cx2d += imgPts[i * 2];
        cy2d += imgPts[i * 2 + 1];
    }
    
    cx3d /= n; cy3d /= n; cz3d /= n;
    cx2d /= n; cy2d /= n;
    
    // Estimate scale from spread of points
    let sum3d = 0, sum2d = 0;
    for (let i = 0; i < n; i++) {
        const dx3d = objPts[i * 3] - cx3d;
        const dy3d = objPts[i * 3 + 1] - cy3d;
        sum3d += Math.sqrt(dx3d * dx3d + dy3d * dy3d);
        
        const dx2d = imgPts[i * 2] - cx2d;
        const dy2d = imgPts[i * 2 + 1] - cy2d;
        sum2d += Math.sqrt(dx2d * dx2d + dy2d * dy2d);
    }
    
    // Better depth estimate from point spread ratio
    const scale3d = sum3d / n;
    const scale2d = sum2d / n;
    const tz = (fx * scale3d) / (scale2d + 1e-6);
    
    // Estimate rotation from point distribution
    let sumX = 0, sumY = 0;
    for (let i = 0; i < n; i++) {
        const dx = (imgPts[i * 2] - cx2d) / fx;
        const dy = (imgPts[i * 2 + 1] - cy2d) / fy;
        sumX += dx * (objPts[i * 3 + 1] - cy3d);
        sumY += dy * (objPts[i * 3] - cx3d);
    }
    
    const ry = Math.atan2(sumY, tz) * 0.5;
    const rx = Math.atan2(-sumX, tz) * 0.5;
    
    return {
        rvec: [rx, ry, 0],
        tvec: [(cx2d - cx) * tz / fx, (cy2d - cy) * tz / fy, tz]
    };
}

// Helper: Refine pose using Levenberg-Marquardt
function refinePose(pose, objPts, imgPts, fx, fy, cx, cy, n, maxIter = 30) {
    let {rvec, tvec} = pose;
    let lambda = 0.001;
    let prevError = Infinity;
    
    for (let iter = 0; iter < maxIter; iter++) {
        const R = rodrigues(rvec);
        const {residuals, J} = computeJacobian(rvec, tvec, R, objPts, imgPts, fx, fy, cx, cy, n);
        
        const error = residuals.reduce((sum, r) => sum + r * r, 0);
        
        // Check convergence
        if (Math.abs(prevError - error) < 1e-8) break;
        if (error < 0.01) break;
        
        // JtJ and Jtr
        const JtJ = new Array(36).fill(0);
        const Jtr = new Array(6).fill(0);
        
        for (let i = 0; i < n * 2; i++) {
            for (let j = 0; j < 6; j++) {
                Jtr[j] += J[i * 6 + j] * residuals[i];
                for (let k = 0; k < 6; k++) {
                    JtJ[j * 6 + k] += J[i * 6 + j] * J[i * 6 + k];
                }
            }
        }
        
        // Add damping
        for (let i = 0; i < 6; i++) {
            JtJ[i * 6 + i] *= (1 + lambda);
        }
        
        // Solve 6x6 system
        const delta = solve6x6(JtJ, Jtr);
        if (!delta) break;
        
        // Check if update is too large
        const deltaSize = Math.sqrt(delta.reduce((s, d) => s + d * d, 0));
        if (deltaSize > 1.0) {
            // Scale down the update
            for (let i = 0; i < 6; i++) delta[i] *= 0.5 / deltaSize;
        }
        
        // Update parameters
        rvec = [rvec[0] - delta[0], rvec[1] - delta[1], rvec[2] - delta[2]];
        tvec = [tvec[0] - delta[3], tvec[1] - delta[4], tvec[2] - delta[5]];
        
        // Adjust damping
        if (error < prevError) {
            lambda *= 0.1;
            prevError = error;
        } else {
            lambda *= 10;
        }
    }
    
    return {rvec, tvec};
}

// Helper: Compute Jacobian and residuals
function computeJacobian(rvec, tvec, R, objPts, imgPts, fx, fy, cx, cy, n) {
    const residuals = [];
    const J = [];
    
    const theta = Math.sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2]);
    
    for (let i = 0; i < n; i++) {
        const X = objPts[i * 3];
        const Y = objPts[i * 3 + 1];
        const Z = objPts[i * 3 + 2];
        
        // Transform point
        const Xc = R[0] * X + R[1] * Y + R[2] * Z + tvec[0];
        const Yc = R[3] * X + R[4] * Y + R[5] * Z + tvec[1];
        const Zc = R[6] * X + R[7] * Y + R[8] * Z + tvec[2];
        
        const invZ = 1 / (Zc + 1e-10);
        const invZ2 = invZ * invZ;
        
        // Project
        const u = fx * Xc * invZ + cx;
        const v = fy * Yc * invZ + cy;
        
        // Residuals
        residuals.push(u - imgPts[i * 2]);
        residuals.push(v - imgPts[i * 2 + 1]);
        
        // Jacobian w.r.t. camera coordinates
        const du_dXc = fx * invZ;
        const du_dYc = 0;
        const du_dZc = -fx * Xc * invZ2;
        
        const dv_dXc = 0;
        const dv_dYc = fy * invZ;
        const dv_dZc = -fy * Yc * invZ2;
        
        // Point in camera frame
        const P = [Xc, Yc, Zc];
        
        // Jacobian of rotation (more accurate derivative)
        const dP_drx = [-P[1] * R[6] + P[2] * R[3], -P[1] * R[7] + P[2] * R[4], -P[1] * R[8] + P[2] * R[5]];
        const dP_dry = [P[0] * R[6] - P[2] * R[0], P[0] * R[7] - P[2] * R[1], P[0] * R[8] - P[2] * R[2]];
        const dP_drz = [-P[0] * R[3] + P[1] * R[0], -P[0] * R[4] + P[1] * R[1], -P[0] * R[5] + P[1] * R[2]];
        
        // Chain rule for u
        const du_drx = du_dXc * dP_drx[0] + du_dYc * dP_drx[1] + du_dZc * dP_drx[2];
        const du_dry = du_dXc * dP_dry[0] + du_dYc * dP_dry[1] + du_dZc * dP_dry[2];
        const du_drz = du_dXc * dP_drz[0] + du_dYc * dP_drz[1] + du_dZc * dP_drz[2];
        
        // Chain rule for v
        const dv_drx = dv_dXc * dP_drx[0] + dv_dYc * dP_drx[1] + dv_dZc * dP_drx[2];
        const dv_dry = dv_dXc * dP_dry[0] + dv_dYc * dP_dry[1] + dv_dZc * dP_dry[2];
        const dv_drz = dv_dXc * dP_drz[0] + dv_dYc * dP_drz[1] + dv_dZc * dP_drz[2];
        
        // Jacobian rows for this point
        J.push(du_drx, du_dry, du_drz, du_dXc, du_dYc, du_dZc);
        J.push(dv_drx, dv_dry, dv_drz, dv_dXc, dv_dYc, dv_dZc);
    }
    
    return {residuals, J};
}

// Helper: Solve 6x6 linear system using Gaussian elimination
function solve6x6(A, b) {
    const n = 6;
    // Convert flat array to 2D matrix
    const M = [];
    for (let i = 0; i < n; i++) {
        M[i] = [];
        for (let j = 0; j < n; j++) {
            M[i][j] = A[i * n + j];
        }
    }
    const B = [...b];
    
    // Forward elimination
    for (let i = 0; i < n; i++) {
        let maxRow = i;
        for (let k = i + 1; k < n; k++) {
            if (Math.abs(M[k][i]) > Math.abs(M[maxRow][i])) maxRow = k;
        }
        [M[i], M[maxRow]] = [M[maxRow], M[i]];
        [B[i], B[maxRow]] = [B[maxRow], B[i]];
        
        if (Math.abs(M[i][i]) < 1e-10) return null;
        
        for (let k = i + 1; k < n; k++) {
            const c = M[k][i] / M[i][i];
            for (let j = i; j < n; j++) {
                M[k][j] -= c * M[i][j];
            }
            B[k] -= c * B[i];
        }
    }
    
    // Back substitution
    const x = new Array(n);
    for (let i = n - 1; i >= 0; i--) {
        x[i] = B[i];
        for (let j = i + 1; j < n; j++) {
            x[i] -= M[i][j] * x[j];
        }
        x[i] /= M[i][i];
    }
    
    return x;
}