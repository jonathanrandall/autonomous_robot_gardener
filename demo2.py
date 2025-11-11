import sys
import cv2
import numpy as np
import ArducamDepthCamera as ac

import subprocess
import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# print(dir(ac))

MAX_DISTANCE = 2000

webcam_name = "Arducam" # "3D USB Camera"# (usb-0000:01:00.0-1.1):"
# webcam_name2 = "3D USB Camera: 3D USB Camera (usb-0000:01:00.0-1.3.4):"

def process_frame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf =(1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    return result_frame 

class UserRect():
    def __init__(self) -> None:
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

selectRect = UserRect()

followRect = UserRect()

def on_mouse(event, x, y, flags, param):
    global selectRect,followRect
    
    if event == cv2.EVENT_LBUTTONDOWN:
        pass

    elif event == cv2.EVENT_LBUTTONUP:
        selectRect.start_x = x - 4 if x - 4 > 0 else 0
        selectRect.start_y = y - 4 if y - 4 > 0 else 0
        selectRect.end_x = x + 4 if x + 4 < 240 else 240
        selectRect.end_y=  y + 4 if y + 4 < 180 else 180
    else:
        followRect.start_x = x - 4 if x - 4 > 0 else 0
        followRect.start_y = y - 4 if y - 4 > 0 else 0
        followRect.end_x = x + 4 if x + 4 < 240 else 240
        followRect.end_y = y + 4 if y + 4 < 180 else 180
        
def usage(argv0):
    print("Usage: python "+argv0+" [options]")
    print("Available options are:")
    print(" -d        Choose the video to use")

def find_webcam_index(device_name):
    command = "v4l2-ctl --list-devices"
    output = subprocess.check_output(command, shell=True, text=True)
    devices = output.split('\n\n')
    
    for device in devices:
        #print(device)
        if device_name in device:
            lines = device.split('\n')
            for line in lines:
                if "video" in line:
                    parts = line.split()
                    for part in parts:
                        if part.startswith('/dev/video'):
                            print(part)
                            return (part[10:])


if __name__ == "__main__":
    cam = ac.ArducamCamera()
    camIDx = int(find_webcam_index("unicam"))
    print("Arducam Depth Camera Demo")
    cfg_path = None
    # cfg_path = "file.cfg"

    black_color = (0, 0, 0)
    white_color = (255, 255, 255)

    ret = 0
    if cfg_path is not None:
        ret = cam.openWithFile(cfg_path, camIDx)
    else:
        ret = cam.open(ac.Connection.CSI, camIDx)
    if ret != 0:
        print("Failed to open camera. Error code:", ret)

    ret = cam.start(ac.FrameType.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        

    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)
    r = cam.getControl(ac.Control.RANGE)
    

    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)
    #cv2.setMouseCallback("preview",on_mouse)
    i=1
    webcam_index = int(find_webcam_index(webcam_name))
    cap_stereo = cv2.VideoCapture(webcam_index)
    cap_stereo.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_stereo.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while True:
        frame = cam.requestFrame(200)
        if not cap_stereo.isOpened():
            print("Error: cannot open")
            break
        ret, stereo_frame = cap_stereo.read()
        stereo_frame = cv2.resize(stereo_frame, (320, 240), interpolation=cv2.INTER_LINEAR)
        
        if frame != None and ret and isinstance(frame, ac.DepthData):
            depth_buf = frame.depth_data
            amplitude_buf = frame.confidence_data
            # depth_buf = frame.getDepthData()
            # amplitude_buf = frame.getAmplitudeData()
            cam.releaseFrame(frame)
            # amplitude_buf*=(255/1024)
            # amplitude_buf = np.clip(amplitude_buf, 0, 255)
            
            cv2.imshow("stereo image", stereo_frame)

            # cv2.imshow("preview_amplitude", amplitude_buf.astype(np.uint8))
            #preview_image = amplitude_buf.copy()
            #preview_image = preview_image.astype(np.uint8)
            #print("select Rect distance:",np.mean(depth_buf[selectRect.start_y:selectRect.end_y,selectRect.start_x:selectRect.end_x]))
            result_image = process_frame(depth_buf,amplitude_buf)
            #result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_JET)
            # cv2.rectangle(result_image,(selectRect.start_x,selectRect.start_y),(selectRect.end_x,selectRect.end_y),(128,128,128), 1)
            # cv2.rectangle(result_image,(followRect.start_x,followRect.start_y),(followRect.end_x,followRect.end_y),(255,255,255), 1)
    
            cv2.imshow("preview",result_image)

            key = cv2.waitKey(1)
            if key == ord("q"):
                exit_ = True
                cam.stop()
                cam.close()
                sys.exit(0)
            if key == ord("s"):
                i=i+1
                fname = os.path.join(BASE_DIR, 'images/tof_'+str(i) + '.jpg')
                fname2 = os.path.join(BASE_DIR, 'images/webcam_'+str(i) + '.jpg')
                # fname = '/home/jonny/images/tof_'+str(i) + '.jpg'
                # fname2 = '/home/jonny/images/stereo_'+str(i) + '.jpg'
                cv2.imwrite(fname2, stereo_frame)
                cv2.imwrite(fname, result_image)
                