import depthai as dai

pipeline = dai.Pipeline()

with dai.Device(pipeline) as device:
    print("connected")