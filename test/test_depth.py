import depthai as dai
import cv2
import numpy as np
import os

os.environ['DEEPFAI_USB2'] = '1'

def create_pipeline():
    # Create DepthAI pipeline
    pipeline = dai.Pipeline()
    mono_left = pipeline.createMonoCamera()
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
    mono_left.setFps(10)
    mono_right = pipeline.createMonoCamera()
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
    mono_right.setFps(10)

    stereo_depth = pipeline.createStereoDepth()
    stereo_depth.setLeftRightCheck(True)
    stereo_depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo_depth.initialConfig.setConfidenceThreshold(200)

    mono_left.out.link(stereo_depth.left)
    mono_right.out.link(stereo_depth.right)

    xout_depth = pipeline.createXLinkOut()
    xout_depth.setStreamName("depth")
    stereo_depth.depth.link(xout_depth.input)

    return pipeline

def get_terrain_height_estimation():
    pipeline = create_pipeline()

    # Get device info to pass to Device constructor
    device_info = dai.DeviceInfo()

    # Set USB speed (USB2 or USB3)
    maxUsbSpeed = dai.UsbSpeed.HIGH  # Use USB2 speed, or you can change this to dai.UsbSpeed.SUPER for USB3

    with dai.Device(pipeline, device_info, maxUsbSpeed) as device:
        q_depth = device.getOutputQueue(name="depth", maxSize=8, blocking=False)

        while True:
            depth_frame = q_depth.get()
            depth_data = depth_frame.getFrame()
            height, width = depth_data.shape
            center_region = depth_data[int(0.4 * height):int(0.6 * height), int(0.4 * width):int(0.6 * width)]
            valid_depths = center_region[center_region > 0]  # Only consider valid depths (> 0)
            
            if valid_depths.size > 0:
                median_depth = np.median(valid_depths)  
                print(f"Estimated terrain height (below robot): {median_depth} mm")

            else:
                print("No valid depth data in the center region")

            depth_normalized = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = depth_normalized.astype('uint8')
            depth_color = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            cv2.imshow("Depth Map", depth_color)
            if cv2.waitKey(1) == 27:  
                break

        cv2.destroyAllWindows()
        device.close()

if __name__ == "__main__":
    get_terrain_height_estimation()
