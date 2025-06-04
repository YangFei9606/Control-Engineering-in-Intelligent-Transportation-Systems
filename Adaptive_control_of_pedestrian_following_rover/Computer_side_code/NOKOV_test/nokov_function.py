import time
import math
import numpy as np
import sys
from nokov.nokovsdk import *

# Global variable to store rigid body ID to name mapping
rigid_body_id_name_map = {}

def quad_to_euler(quad_info):
    # Conversion formula
    qx = quad_info[0]
    qy = quad_info[1]
    qz = quad_info[2]
    qw = quad_info[3]

    t0 = +2.0 * (qw * qx + qy * qz)
    t1 = +1.0 - 2.0 * (qx * qx + qy * qy)
    roll_x = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (qw * qy - qz * qx)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch_y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_z = np.degrees(np.arctan2(t3, t4))

    # Adjusting the range from 0 to 360 degrees
    roll_x = (roll_x + 360) % 360
    pitch_y = (pitch_y + 360) % 360
    yaw_z = (yaw_z + 360) % 360

    return [roll_x, pitch_y, yaw_z]

def nokov_setup():
    global rigid_body_id_name_map
    client = PySDKClient()

    serverIp = '10.1.1.198'  # Replace with your server IP

    ret = client.Initialize(bytes(serverIp, encoding="utf8"))
    if ret == 0:
        print("Connected to the Nokov server successfully.")
    else:
        print(f"Failed to connect to the Nokov server: [{ret}]")
        sys.exit(0)

    # Get data descriptions to build rigid body ID-name mapping
    pdds = POINTER(DataDescriptions)()
    ret = client.PyGetDataDescriptions(pdds)
    if ret != 0:
        print("Failed to get data descriptions.")
        sys.exit(0)

    dataDefs = pdds.contents
    for iDef in range(dataDefs.nDataDescriptions):
        dataDef = dataDefs.arrDataDescriptions[iDef]
        if dataDef.type == DataDescriptors.Descriptor_RigidBody.value:
            rigidBody = dataDef.Data.RigidBodyDescription.contents
            rigid_body_id_name_map[rigidBody.ID] = rigidBody.szName.decode('utf-8')
            print(f"RigidBody ID: {rigidBody.ID}, Name: {rigid_body_id_name_map[rigidBody.ID]}")

    print("Setup is completed!")
    return client

def nokov_read_data(client, BodyName):
    frame = client.PyGetLastFrameOfMocapData()
    if frame:
        frameData = frame.contents
        for iBody in range(frameData.nRigidBodies):
            body = frameData.RigidBodies[iBody]
            rigid_body_id = body.ID
            if rigid_body_id in rigid_body_id_name_map:
                rigid_body_name = rigid_body_id_name_map[rigid_body_id]
                if rigid_body_name == BodyName:
                    # Extract position and quaternion
                    pos_info = [body.x, body.y, body.z]
                    quad_info = [body.qx, body.qy, body.qz, body.qw]
                    # print(f"RigidBody '{rigid_body_name}' found.")
                    # print(f"Position: {pos_info}")
                    # print(f"Quaternion: {quad_info}")
                    client.PyNokovFreeFrame(frame)
                    return pos_info, quad_info
    else:
        print("No frame data received.")
    client.PyNokovFreeFrame(frame)
    return None, None

def nokov_feedback(client, Bodyname, pos_now):

    pos_info, quad_info = nokov_read_data(client, Bodyname)

    if pos_info and quad_info:
        ang_info = quad_to_euler(quad_info)
        yaw_angle = ang_info[2]/180 * math.pi
        print(f"Yaw angle: {yaw_angle}")
        # Y coordinate is the output
        pos_next = np.matrix([pos_info[0]/1000, pos_info[1]/1000, yaw_angle])
        # Y coordinate is the negative of output
        # pos_next = np.matrix([pos_info[0]/1000, -pos_info[1]/1000, ang_info[2]])
        # print(f"Current rover position: {pos_next}")
        return pos_next
    else:
        print(f"RigidBody {Bodyname} not found in the current frame.")
        return pos_now

