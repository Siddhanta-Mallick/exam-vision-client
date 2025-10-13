import { useRef, useEffect, useState } from "react";
import { FilesetResolver, FaceLandmarker } from "@mediapipe/tasks-vision";
import { computeHeadPose } from "../utils/headPoseUtils.js";

export default function ProctorMainWindow() {

    const KEY_LANDMARK_INDICES = [1, 9, 57, 130, 287, 359];

    const videoRef = useRef(null);
    const faceLandmarkerRef = useRef(null);
    const [faceAngles, setFaceAngles] = useState({
        pitch: 0,
        yaw: 0,
        roll: 0
    });

    useEffect(() => {
        const init = async () => {
            // 1. Load wasm files
            const vision = await FilesetResolver.forVisionTasks(
                "https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision/wasm"
            );

            // 2. Init FaceLandmarker with VIDEO mode
            const faceLandmarker = await FaceLandmarker.createFromOptions(vision, {
                baseOptions: {
                    modelAssetPath:
                        "https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task",
                },
                runningMode: "VIDEO",
            });

            faceLandmarkerRef.current = faceLandmarker;

            // 3. Init camera
            const stream = await navigator.mediaDevices.getUserMedia({
                video: {
                    width: { ideal: 640 },
                    height: { ideal: 480 }
                }
            });
            if (videoRef.current) {
                videoRef.current.srcObject = stream;
                videoRef.current.onloadeddata = () => {
                    requestAnimationFrame(detectFace);
                };
            }
        };

        init();
    }, []);

    // 3. Frame loop for detection
    const detectFace = async (timestamp) => {
        if (
            faceLandmarkerRef.current &&
            videoRef.current &&
            !videoRef.current.paused &&
            !videoRef.current.ended
        ) {
            const results = faceLandmarkerRef.current.detectForVideo(
                videoRef.current,
                timestamp
            );

            if (results.faceLandmarks[0]) {
                let key_landmarks = KEY_LANDMARK_INDICES.map(key_index => results.faceLandmarks[0][key_index]);
                let result = await computeHeadPose(key_landmarks)
                // console.log("Key Landmarks:", key_landmarks);
                setFaceAngles({
                    pitch: result.rotation.pitch * 100,
                    yaw: (result.rotation.yaw * 100) - 10,
                    roll: result.rotation.roll * 100
                })
            } else {
                setFaceAngles({
                    pitch: "Face Not Found",
                    yaw: "Face Not Found",
                    roll: "Face Not Found"
                })
            }
        }
        requestAnimationFrame(detectFace);
    };

    return (
        <div className="w-full h-full">
            <div className={`w-full h-full ${faceAngles.yaw > 30 || faceAngles.yaw < -30 || faceAngles.yaw === "Face Not Found" ? "bg-red-500" : ""} flex flex-row justify-around items-center`}>
                <div className="p-10">
                    Inference :
                    Focused
                </div>
                <div className="w-auto p-10">
                    <video ref={videoRef} autoPlay playsInline muted width={640} height={480} />
                </div>
                <div className="p-10">
                    Capture Results :
                    <h1>Pitch : {faceAngles.pitch}</h1>
                    <h1>Yaw : {faceAngles.yaw}</h1>
                    <h1>Roll : {faceAngles.roll}</h1>
                </div>
            </div>
        </div>
    );
}