import { useRef, useEffect, useState } from "react";
import { FilesetResolver, FaceLandmarker } from "@mediapipe/tasks-vision";
import { computeHeadPose } from "../utils/headPoseUtils.js";

function truncateDecimals(number, digits) {
    let signCorrection = number < 0 ? -1 : 1;
    let number_abs = Math.abs(number)
    number_abs *= 10**digits
    number_abs = Math.trunc(number_abs)
    return number_abs / (10**digits) * signCorrection

};

export default function ProctorMainWindow() {

    const FACE_NOT_FOUND_MESSAGE = "Face Not Found"

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
                setFaceAngles({
                    pitch: String(Math.trunc(result.rotation.pitch * 100)).padEnd(FACE_NOT_FOUND_MESSAGE.length, " "),
                    yaw: String((Math.trunc(result.rotation.yaw * 100) - 10)).padEnd(FACE_NOT_FOUND_MESSAGE.length, " "),
                    roll: String(Math.trunc(result.rotation.roll * 100)).padEnd(FACE_NOT_FOUND_MESSAGE.length, " ")
                })
            } else {
                setFaceAngles({
                    pitch: FACE_NOT_FOUND_MESSAGE,
                    yaw: FACE_NOT_FOUND_MESSAGE,
                    roll: FACE_NOT_FOUND_MESSAGE
                })
            }
        }
        requestAnimationFrame(detectFace);
    };

    return (
        <div className="w-full h-full">
            <div className={`
            flex flex-row justify-around items-center
            w-full h-full 
            ${faceAngles.yaw > 30 || faceAngles.yaw < -30 || faceAngles.yaw === "Face Not Found" ? "bg-red-500" : ""} `}>
                <div className="
                flex flex-col items-center gap-20 flex-1
                h-full
                pt-40">
                    <div className="text-6xl">Inference : </div>
                    <div className="text-4xl">Looking xyz</div>
                </div>
                <div className="
                flex flex-2 justify-center
                w-full h-full">
                    <video ref={videoRef} autoPlay playsInline muted width={640} height={480} />
                </div>
                <div className="flex flex-col gap-20 flex-1
                h-full
                pt-40">
                    <div className="text-6xl">Capture :</div>
                    <div className="text-4xl">
                        <h1>Pitch : {faceAngles.pitch}</h1>
                        <h1>Yaw : {faceAngles.yaw}</h1>
                        <h1>Roll : {faceAngles.roll}</h1>
                    </div>
                </div>
            </div>
        </div>
    );
}