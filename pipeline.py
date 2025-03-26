#!/usr/bin/env python3

"""
The code is the same as for Tiny Yolo V3 and V4, the only difference is the blob file
- Tiny YOLOv3: https://github.com/david8862/keras-YOLOv3-model-set
- Tiny YOLOv4: https://github.com/TNTWEN/OpenVINO-YOLOV4
"""

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import open3d as o3d

# Blob path
nnBlobPath = str((Path(__file__).parent / Path('yolov8ntrained_openvino_2022.1_5shave.blob')).resolve().absolute())

FPS = 12
class FPSCounter:
    def __init__(self):
        self.frameCount = 0
        self.fps = 0
        self.startTime = time.time()

    def tick(self):
        self.frameCount += 1
        if self.frameCount % 10 == 0:
            elapsedTime = time.time() - self.startTime
            self.fps = self.frameCount / elapsedTime
            self.frameCount = 0
            self.startTime = time.time()
        return self.fps
    
if not Path(nnBlobPath).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

labelMap = ["blockage", "blowout", "crack"]

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
pointcloud = pipeline.create(dai.node.PointCloud)
nnNetworkOut = pipeline.create(dai.node.XLinkOut)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutPoint= pipeline.create(dai.node.XLinkOut)


xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutPoint.setStreamName("pc")
nnNetworkOut.setStreamName("nnNetwork")


camRgb.setPreviewSize(640, 640)
camRgb.setFps(FPS)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoLeft.setFps(FPS)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")
monoRight.setFps(FPS)


stereo.initialConfig.setLeftRightCheckThreshold(5)
stereo.initialConfig.setConfidenceThreshold(5)
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
stereo.setLeftRightCheck(True)
stereo.setExtendedDisparity(False)
stereo.setSubpixel(False)


# setting node configs

# Align depth map to the perspective of RGB camera, on which inference is done
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())


spatialDetectionNetwork.setBlobPath(nnBlobPath)
spatialDetectionNetwork.setConfidenceThreshold(0.3)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# Yolo specific parameters
spatialDetectionNetwork.setNumClasses(3)
spatialDetectionNetwork.setCoordinateSize(4)
spatialDetectionNetwork.setAnchors([10,14, 23,27, 37,58, 81,82, 135,169, 344,319])
spatialDetectionNetwork.setAnchorMasks({ "side26": [1,2,3], "side13": [3,4,5] })
spatialDetectionNetwork.setIouThreshold(0.5)

labelMap = [
    "blockage",
    "blowout",
    "crack"
]

spatialDetectionNetwork.setNumNCEPerInferenceThread(2)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
spatialDetectionNetwork.passthrough.link(xoutRgb.input)

spatialDetectionNetwork.out.link(xoutNN.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
stereo.depth.link(pointcloud.inputDepth)

spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)
pointcloud.outputPointCloud.link(xoutPoint.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    isRunning = True
    def key_callback(vis, action, mods):
        global isRunning
        if action == 0:
            isRunning = False

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above


    previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
    pcQueue = device.getOutputQueue(name="pc", maxSize=4, blocking=False)
    networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=4, blocking=False)


    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window()
    vis.register_key_action_callback(81, key_callback)
    pcd = o3d.geometry.PointCloud()
    coordinateFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[0,0,0])
    vis.add_geometry(coordinateFrame)

    first = True
    fpsCounter = FPSCounter()
   
    circle = o3d.geometry.TriangleMesh.create_cylinder(100, 1)

    initialization_counter = 0

    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)
    printOutputLayersOnce = True

    while True:

  

        inMessage = pcQueue.get()
       # inColor = inMessage["rgb"]
        inPointCloud = inMessage
        

        inPreview = previewQueue.get()
    
        inDet = detectionNNQueue.get()

        # if None in (inMessage, inDet, pcQueue):
        #     continue



# neural network fuckers 
#        if printOutputLayersOnce:
#            toPrint = 'Output layer names:'
#            for ten in inNN.getAllLayerNames():
#                toPrint = f'{toPrint} {ten},'
#            print(toPrint)
#            printOutputLayersOnce = False

        frame = inPreview.getCvFrame()

#        counter+=1
#        current_time = time.monotonic()
#        if (current_time - startTime) > 1 :
#            fps = counter / (current_time - startTime)
#            counter = 0
#            startTime = current_time

        detections = inDet.detections

        # If the frame is available, draw bounding boxes on it and show the frame

        
#        cvRGBFrame = cv2.cvtColor(cvCaolorFrame, cv2.COLOR_BGR2RGB)


        points = inPointCloud.getPoints().astype(np.float64)

        # Remove zero points
        non_zero_mask = ~np.all(points == 0, axis=1)
        points = points[non_zero_mask]

        # Filter based on distances
        distances = points[:, 2]
        counts, bin_edges = np.histogram(distances, bins=100, density=False)
        # changes = [counts[i] - counts[i + 1] for i in range(len(counts) - 1)]
        # max_dist = bin_edges[np.argmax(changes[1:])+2]
        max_dist = bin_edges[2]
                    # Apply max_dist filtering
        valid_points_mask = points[:, 2] <= max_dist
        points = points[valid_points_mask]

        # Filter based on radius
        # check = points[((points[:,2] <= bin_edges[1]) &  (points[:,2] >= bin_edges[0]))]
        check = points
        q1_points = check[(check[:,0]>0) & (check[:,1]>0)]
        q1_centre = np.array((np.average(q1_points[:,0]), np.average(q1_points[:,1])))
        q2_points = check[(check[:,0]<0) & (check[:,1]>0)]
        q2_centre = np.array((np.average(q2_points[:,0]), np.average(q2_points[:,1])))
        q3_points = check[(check[:,0]<0) & (check[:,1]<0)]
        q3_centre = np.array((np.average(q3_points[:,0]), np.average(q3_points[:,1])))
        q4_points = check[(check[:,0]>0) & (check[:,1]<0)]
        q4_centre = np.array((np.average(q4_points[:,0]), np.average(q4_points[:,1])))
        centre = (q1_centre+q2_centre+q3_centre+q4_centre)/4
        radial_distance = np.sqrt((points[:,0] - centre[0]) ** 2 + (points[:,1] - centre[1]) ** 2)
        radius_mask = [1] * len(points)
        try:
            rad_count, rad_bin_edges = np.histogram(radial_distance, bins=100, density=False)

            # Compute differences between consecutive bins
            diff_counts = np.diff(rad_count)

            # Define a threshold for a "big jump"
            threshold = np.mean(diff_counts) + np.std(diff_counts)  # Mean + 2*std deviation

            # Find indices where the jump is "big"
            big_jump_indices = np.where(diff_counts > threshold)[0]  # Get indices where jump is large

            # Get the first two "big jumps" in order
            if len(big_jump_indices) >= 2:
                first_jump_idx, second_jump_idx = big_jump_indices[:2]
                first_jump_bin = rad_bin_edges[first_jump_idx - 1]
                second_jump_bin = rad_bin_edges[second_jump_idx + 3]

            # avg_rad = (first_jump_bin + second_jump_bin) /2
            radius_mask = radial_distance <= second_jump_bin
            print(big_jump_indices)
            print(second_jump_bin)
            print()
            points = points[radius_mask]

            # Ensure colors are filtered accordingly
            colors = cv2.resize(frame, (640, 400))
            colors = colors.reshape(-1, 3) / 255.0

            # Apply your point-cloud masks now
            colors = colors[non_zero_mask]
            colors = colors[valid_points_mask]
            colors = colors[radius_mask]
            # points = inPointCloud.getPoints().astype(np.float64)
            # points = points[((points[:,2] <= bin_edges[2]) &  (points[:,2] >= bin_edges[0]))]
            # plt.figure()
            # plt.scatter(points[:,0], points[:,1])
            # plt.show()
            # Convert to Open3D format
        except:
            pass
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        if first:
            vis.add_geometry(pcd)
            vis.add_geometry(circle)
            first = False
        else:
            pcd.clear()  # Clear previous geometry explicitly
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            vis.update_geometry(pcd)


        height = frame.shape[0]
        width  = frame.shape[1]
        for detection in detections:
            x1 = int(detection.xmin * width)
            y1 = int(detection.ymin * height)
            x2 = int(detection.xmax * width)
            y2 = int(detection.ymax * height)


            # Define colors
            label_color = (0, 255, 255)     # Yellow for label
            label_bg_color = (0, 0, 0)      # Black background for highlight
            text_color = (0, 0, 255)   # Red for coordinates

            # Draw label with background
            label_text = f"{labelMap[detection.label]}"
            (text_width, text_height), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_TRIPLEX, 1.0, 2)
            cv2.rectangle(frame, (x1 + 30, y1 + 15 - text_height), (x1 + 10 + text_width, y1 + 15 + 5), label_bg_color, -1)
            cv2.putText(frame, label_text, (x1 + 30, y1 + 15), cv2.FONT_HERSHEY_TRIPLEX, 1.0, label_color, 2)

            # Draw coordinates in red
            cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 25, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, text_color, 2)
            cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 25, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, text_color, 2)
            cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 25, y1 + 110), cv2.FONT_HERSHEY_TRIPLEX, 0.5, text_color, 2)

            # Draw rectangle
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            



         # or any larger size you want

        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
        cv2.namedWindow("rgb", cv2.WINDOW_NORMAL)

        cv2.imshow("rgb", frame)


       


        # Convert the frame to RGB

        vis.poll_events()
        vis.update_renderer()
        if cv2.waitKey(1) == ord('q'):
            break

    vis.destroy_window()
