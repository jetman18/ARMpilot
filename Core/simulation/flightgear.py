"""
Start FlightGear with:
`fgfs --native-fdm=socket,out,30,localhost,5501,udp --native-ctrls=socket,out,30,localhost,5503,udp --native-ctrls=socket,in,30,localhost,5504,udp`
"""
import time
import math
from flightgear_python.fg_if import FDMConnection, CtrlsConnection
import multiprocessing as mp
import struct

def ctrls_callback(ctrls_data, event_pipe):
    if event_pipe.child_poll():
        ail_ctrl,ele_ctrl,rud_ctrl,thro_ctrl,flap_ctrl = event_pipe.child_recv()  
        ctrls_data.elevator = ele_ctrl
        ctrls_data.aileron  = ail_ctrl
        ctrls_data.rudder =   rud_ctrl
        ctrls_data.throttle[0] = 1#thro_ctrl
        #ctrls_data.throttle[1] = 1#thro_ctrl
        #ctrls_data.flaps = flap_ctrl
        return ctrls_data

def fdm_callback(fdm_data, event_pipe):
    buffer =[]
    buffer.append(math.degrees(fdm_data.phi_rad))            # roll       (deg)
    buffer.append(math.degrees(fdm_data.theta_rad))          # pitch      (deg)
    buffer.append( math.degrees(fdm_data.psi_rad))           # yaw        (deg)
    buffer.append(math.degrees(fdm_data.lat_rad))            # latitude   (deg)
    buffer.append(math.degrees(fdm_data.lon_rad))            # Longitude  (deg)
    buffer.append(fdm_data.alt_m)                            # Altitude    (m)
    buffer.append(fdm_data.eng_state[0])                     # Engine states
    buffer.append(math.degrees(fdm_data.psidot_rad_per_s))
    buffer.append(fdm_data.rpm[0])    
    buffer.append(fdm_data.v_body_u)                              # Engine rpm
    event_pipe.child_send((buffer))
    
    if event_pipe.child_poll():
        recvFDM =[]
        recvFDM = event_pipe.recv()
        #fdm_data.alt_m = 0
        return fdm_data
    
if __name__ == '__main__':  # NOTE: This is REQUIRED on Windows!
    ctrls_conn = CtrlsConnection(ctrls_version=27)
    ctrls_event_pipe = ctrls_conn.connect_rx('localhost', 5503, ctrls_callback)
    ctrls_conn.connect_tx('localhost', 5504)
    
    fdm_conn = FDMConnection(fdm_version=24)  # May need to change version from 24
    fdm_event_pipe = fdm_conn.connect_rx('localhost', 5501, fdm_callback)
    fdm_conn.connect_tx('localhost', 5502)

    ctrls_conn.start() 
    fdm_conn.start()  

    timer = time.time()


    p_lat = 0
    p_lon = 0
    roll_fmd = 0
    pitch_fmd = 0
    yaw_fmd   = 0
    latitude  = 0
    longitude = 0
    altitude  = 0
    engine_rpm = 0
    v_body_u   = 0
    recvAtitude=[]   # remove ?
    # Temporary variable


    while 1:  
        recvAtitude = fdm_event_pipe.parent_recv() 
        if  time.time() - timer > 0.1:  # 10hz loop
            timer = time.time()
            roll_fmd  = recvAtitude[0]   # deg
            pitch_fmd = recvAtitude[1]   # deg
            yaw_fmd   = recvAtitude[2]   # deg
            latitude  = recvAtitude[3]   # deg
            longitude = recvAtitude[4]   # deg
            altitude  = recvAtitude[5]   # m
            engine_rpm = recvAtitude[8]  # RPM
            v_body_u   = recvAtitude[9]  # m/s
 
   
                
            ctrls_event_pipe.parent_send((0,0,0,0,0))
            #fdm_event_pipe.parent_send((0,))  # send tuple
            ctrls_event_pipe.clear() #important
    # stop all child process
    ctrls_conn.stop()
    fdm_conn.stop()
    

