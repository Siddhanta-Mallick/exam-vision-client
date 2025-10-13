import { solvePnP, rodrigues } from './pnp-rodrigues.js';

export async function computeHeadPose(faceCoordinationInImageArray, imageWidth = 640, imageHeight = 480) {
    // No loading needed - instant execution!
    
    // 3D model points for key facial landmarks (in mm)
    // Using a more symmetric model
    const facialCoordinationInRealWorld =[
        285, 528, 200,        // Nose tip (landmark 1)
        285, 371, 152,    // Chin (landmark 9)
        197, 574, 128,   // Left eye left corner (landmark 130)
        173, 425, 108,  // Left mouth corner (landmark 57)
        360, 574, 128,   // Right mouth corner (landmark 287)
        391, 425, 108     // Right eye right corner (landmark 359)
    ];
    // const facialCoordinationInRealWorld =[
    //     0.0, 0.0, 0.0,        // Nose tip (landmark 1)
    //     0.0, -63.6, -12.5,    // Chin (landmark 9)
    //     -43.3, 32.7, -26.0,   // Left eye left corner (landmark 130)
    //     -28.9, -28.9, -24.1,  // Left mouth corner (landmark 57)
    //     28.9, -28.9, -24.1,   // Right mouth corner (landmark 287)
    //     43.3, 32.7, -26.0     // Right eye right corner (landmark 359)
    // ];
    
    // Convert normalized coordinates to pixel coordinates
    const pixelCoords = [];
    for (let i = 0; i < faceCoordinationInImageArray.length; i++) {
        const landmark = faceCoordinationInImageArray[i];
        pixelCoords.push(landmark.x * imageWidth);
        pixelCoords.push(landmark.y * imageHeight);
    }
    
    const focalLength = 1 * imageWidth;
    const cameraMatrix = [
        focalLength, 0, imageWidth / 2,
        0, focalLength, imageHeight / 2,
        0, 0, 1,
    ];

    // Solve PnP
    const {rvec, tvec} = solvePnP(
        facialCoordinationInRealWorld,
        pixelCoords,
        cameraMatrix
    );

    // Convert rotation vector to rotation matrix
    const R = rodrigues(rvec);

    // Euler angles (degrees)
    const euler = rotationMatrixToEulerAngles(R);

    return {
        rotation: euler,
    };
}

function rotationMatrixToEulerAngles(R) {
    const sy = Math.sqrt(R[0] * R[0] + R[3] * R[3]);
    let pitch, yaw, roll;

    if (sy > 1e-6) {
        pitch = Math.atan2(R[7], R[8]);  // Rotation around X-axis
        yaw = Math.atan2(-R[6], sy);     // Rotation around Y-axis
        roll = Math.atan2(R[3], R[0]);   // Rotation around Z-axis
    } else {
        pitch = Math.atan2(-R[5], R[4]);
        yaw = Math.atan2(-R[6], sy);
        roll = 0;
    }

    return {
        pitch: (pitch * 180) / Math.PI,  // Up/Down (nodding)
        yaw: (yaw * 180) / Math.PI,      // Left/Right (shaking head)
        roll: (roll * 180) / Math.PI,    // Tilt (ear to shoulder)
    };
}