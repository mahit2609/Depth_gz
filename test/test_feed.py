import depthai as dai
import cv2
import os

os.environ['DEEPFAI_USB2'] = '1'

def initialize_device():
    try:
        # Create a pipeline object
        pipeline = dai.Pipeline()

        # Create Color Camera node
        color_camera = pipeline.createColorCamera()
        color_camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        color_camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        color_camera.setFps(10)

        # Create output for color camera stream
        xout = pipeline.createXLinkOut()
        xout.setStreamName("video")
        color_camera.video.link(xout.input)

        # Get device information (this is necessary for initialization)
        device_info = dai.DeviceInfo()

        # Set USB speed (USB2 or USB3)
        maxUsbSpeed = dai.UsbSpeed.HIGH  # USB2 (low speed) or SUPER for USB3

        # Initialize the device with the pipeline, device information, and USB speed
        device = dai.Device(pipeline, device_info, maxUsbSpeed)

        print("Device initialized successfully")
        return device

    except Exception as e:
        print(f"Error initializing device: {e}")
        return None

def capture_video():
    device = initialize_device()
    if not device:
        print("Failed to initialize device. Exiting...")
        return

    # Output queue for color frames
    q = device.getOutputQueue(name="video", maxSize=8, blocking=False)

    while True:
        try:
            frame = q.get()  # Get the frame from the output queue
            frame = frame.getCvFrame()  # Convert the frame to OpenCV format

            # Show the frame in a window
            cv2.imshow("Color Camera", frame)

            # Check for exit key press (ESC)
            if cv2.waitKey(1) == 27:  # 27 is the ESC key
                break
        except Exception as e:
            print(f"Error during frame capture: {e}")
            break

    cv2.destroyAllWindows()
    device.close()  # Make sure to properly close the device after use

if __name__ == "__main__":
    capture_video()
