from pymavlink import mavutil
import requests
import time
import struct
import logging
import threading
import queue


PARAMS_URL = "https://caseparams.sandbox.aviant.no/reference.params"
MAVLINK_CONNECTION_URL = "/dev/tty.usbmodem01"
BAUD = 57600
MAV_PARAM_TYPE = {
    1 : int,
    2 : int,
    3 : int,
    4 : int,
    5 : int,
    6 : int,
    7 : int,
    8 : int,
    9 : float,
    10 : float,
}

MAV_PARAM_FORMAT_TYPE = {
    1 : 'B',
    2 : 'b',
    3 : 'H',
    4 : 'h',
    5 : 'I',
    6 : 'i',
    7 : 'Q',
    8 : 'q',
    9 : 'f',
    10 : 'd',
}

REF_PARAMS_REFRESH_PERIOD = 1 # s
DRONE_PARAMS_REFRESH_PERIOD = 10 # s

        
def get_reference_params():
    response = requests.get(PARAMS_URL)
    if response.status_code == 200:
        return response.text


def parse_params_file(file):
    lines = file.strip().split('\n')
    slice = 0
    while True: # find index of first line that is not a header
        if lines[slice][0] != "#":
            break
        slice += 1
    values = [line.split('\t')[2:] for line in lines[slice:]] # ignore vehicle & component id
    key_value_pairs = {}
    for value in values:
        key_value_pairs[value[0]] = {
            "Value" : MAV_PARAM_TYPE[int(value[2])](value[1]),
            "Type"  : int(value[2])
        } # convert values to proper data type
    return key_value_pairs


def refresh_reference_params():
    text = get_reference_params()
    return parse_params_file(text)


def get_params_on_drone(connection):
    connection.mav.param_request_list_send(
        connection.target_system,
        connection.target_component
    )
    params = {}
    while True:
        message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if message is None:
            break
        #print(message.param_value)
        value = struct.unpack(MAV_PARAM_FORMAT_TYPE[message.param_type], struct.pack('f', message.param_value))[0] # typeprune from float
        #print(value)
        params[message.param_id] = {
            "Value" : value,
            "Type"  : message.param_type
        }
        
    return params


def compare_param_list(reference_params, drone_params):
    last_match_idx = 0
    diff = {}
    unrecognized_params = {}
    found_match = False
    for ref_param, ref_value in reference_params.items():
        for idx, drone_param in enumerate(list(drone_params.keys())[last_match_idx:]):
            if ref_param == drone_param:
                last_match_idx = idx + 1
                drone_value = drone_params[drone_param]
                if ref_value != drone_value:
                    diff[ref_param] = ref_value
                found_match = True
                break
        if not found_match:
            unrecognized_params[ref_param] = ref_value
        found_match = False
    return diff, unrecognized_params


def update_drone_params(params, connection, connection_read_lock):
    unacknowlaged = []
    for param, value in params.items():
        print(value["Value"])
        packed_value = struct.unpack('f', struct.pack(MAV_PARAM_FORMAT_TYPE[value["Type"]], value["Value"]))[0] # typeprune to float
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            param.encode("utf-8"),
            packed_value,
            value["Type"]
        )
        with connection_read_lock:
            message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if not message or message.param_id.strip() != param:
                unacknowlaged.append(param)
    return unacknowlaged

def create_QGC_sync_message(diff, unrecognized_params, unsynced_params):
    message = "Synced parameters from online reference. The following parameters were updated with new values:\n\n"
    for param, value in diff.items():
        message += f"  {param}: {value['Value']}\n"
    if unrecognized_params:
        message += "\nFailed to sync the following parameters because they were not present on drone:\n\n"
        for param, value in unrecognized_params.items():
            message += f"  {param}: {value['Value']}\n"
    if unsynced_params:
        message += "\nThe following parameters were not acknowledged after sync:\n\n"
        for param, value in unsynced_params.items():
            message += f"  {param}: {value['Value']}\n"
    return message 

                    
def main():
    connection = mavutil.mavlink_connection(MAVLINK_CONNECTION_URL, baud=BAUD)
    if not connection:
        print("Failed to get connection URL")
        return
    print(f"Mavlink connection established at:{MAVLINK_CONNECTION_URL}")
    print("Waiting for heartbeat...")
    heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
    while not heartbeat:
        print("Timed out waiting for heartbeat, trying again...")
        heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
    print("Got heartbeat")
    drone_params = get_params_on_drone(connection)
    reference_params = get_reference_params()
    reference_params = parse_params_file(reference_params)
    ref_param_update_time = time.perf_counter()
    drone_param_update_time = time.perf_counter()
    armed = heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    while True:
        if not armed:
            diff = {}
            refresh = False

            if time.perf_counter() - ref_param_update_time > REF_PARAMS_REFRESH_PERIOD:
                print("refresh ref params")
                reference_params = get_reference_params()
                reference_params = parse_params_file(reference_params)
                ref_param_update_time = time.perf_counter()
                refresh = True
            
            if time.perf_counter() - drone_param_update_time > DRONE_PARAMS_REFRESH_PERIOD:
                print("refresh drone params")
                drone_params = get_params_on_drone(connection)
                drone_param_update_time = time.perf_counter()
                refresh = True

            if refresh:
                diff, unrecognized_params = compare_param_list(reference_params, drone_params)
                print(diff)
            
            if diff:
                #update_drone_params(diff)
                message = create_QGC_sync_message(diff, unrecognized_params)
                drone_param_update_time = time.perf_counter()
                for param, value in diff.items():
                    drone_params[param] = value
                connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, message.encode("utf-8"))

        heartbeat = connection.recv_match(type="HEARTBEAT", blocking=False)
        if heartbeat:
            print("Got heartbeat")
            armed = heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


class FetchRefParamsThread(threading.Thread):
    def __init__(self, arm_event, stage_event, stop_event, staged_changes, unrecognized_params, drone_params, drone_params_lock):
        threading.Thread.__init__(self)
        self.arm_event = arm_event
        self.stage_event = stage_event
        self.stop_event = stop_event
        self.staged_changes = staged_changes
        self.unrecognized_params = unrecognized_params
        self.drone_params = drone_params
        self.drone_params_lock = drone_params_lock

    def run(self):
        while not self.stop_event.wait(REF_PARAMS_REFRESH_PERIOD):
            if not self.arm_event.is_set():
                with self.drone_params_lock:
                    reference_params = get_reference_params()
                    reference_params = parse_params_file(reference_params)
                    diff, unrecognized_params = compare_param_list(reference_params, self.drone_params)
                    print("params fetched")
                    if diff:
                        self.staged_changes.put(diff)
                        self.unrecognized_params.put(unrecognized_params)
                        self.stage_event.set()


class HeartbeatThread(threading.Thread):
    def __init__(self, connection, connection_lock, arm_event, stop_event):
        threading.Thread.__init__(self)
        self.connection = connection
        self.connection_lock = connection_lock
        self.arm_event = arm_event
        self.stop_event = stop_event

    def run(self):
        while not self.stop_event.is_set():
            time.sleep(1) # avoiding hogging the connection read lock
            with self.connection_lock:
                heartbeat = self.connection.recv_match(type="HEARTBEAT", blocking=False)
                if heartbeat:
                    print("Got heartbeat")
                    armed = heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    if self.arm_event.is_set() and not armed:
                        self.arm_event.clear()
                    elif not self.arm_event.is_set() and armed:
                        self.arm_event.set()


class ParamUpdateThread(threading.Thread):
    def __init__(self, stage_event, arm_event, stop_event, connection, connection_write_lock, connection_read_lock, staged_changes, unrecognized_params):
        threading.Thread.__init__(self)
        self.connection = connection
        self.connection_write_lock = connection_write_lock
        self.connection_read_lock = connection_read_lock
        self.stage_event = stage_event
        self.arm_event = arm_event
        self.stop_event = stop_event
        self.staged_changes = staged_changes
        self.unrecognized_params = unrecognized_params

    def run(self):
        while not self.stop_event.is_set():
            if self.stage_event.is_set() and not self.arm_event.is_set():
                print("Change staged")
                with self.connection_write_lock:
                    diff = self.staged_changes.get()
                    diff = {'BAT1_N_CELLS': {"Value": 6, "Type" : 6}}
                    unrecognized = self.unrecognized_params.get()
                    print(update_drone_params(diff, self.connection, self.connection_read_lock))
                    unsynced = []
                    print("updated params")
                    
                    qgc_message = create_QGC_sync_message(diff, unrecognized, unsynced)
                    self.connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, qgc_message.encode("utf-8"))
                    self.stage_event.clear()


def main_thread():
    connection = mavutil.mavlink_connection(MAVLINK_CONNECTION_URL, baud=BAUD)
    if not connection:
        print("Failed to get mavlink connection")
        return
    print(f"Mavlink connection established at:{MAVLINK_CONNECTION_URL}")
    print("Waiting for heartbeat...")
    heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
    while not heartbeat:
        print("Timed out waiting for heartbeat, trying again...")
        heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
    print("Got heartbeat")
    drone_params = get_params_on_drone(connection)
    reference_params = get_reference_params()
    reference_params = parse_params_file(reference_params)

    armed = threading.Event()
    staged = threading.Event()
    stop = threading.Event()

    staged_changes = queue.Queue()
    unrecognized_params = queue.Queue()

    connection_read_lock = threading.Lock()
    connection_write_lock = threading.Lock()
    drone_param_lock = threading.Lock()

    heartbeat_thread = HeartbeatThread(connection=connection,
                                       connection_lock=connection_read_lock, 
                                       arm_event=armed, 
                                       stop_event=stop)
    refparam_thread = FetchRefParamsThread(arm_event=armed, 
                                          stage_event=staged, 
                                          stop_event=stop, 
                                          staged_changes=staged_changes, 
                                          unrecognized_params=unrecognized_params,
                                          drone_params=drone_params,
                                          drone_params_lock=drone_param_lock)
    update_params_thread = ParamUpdateThread(connection=connection,
                                             connection_write_lock=connection_write_lock,
                                             connection_read_lock=connection_read_lock,
                                             arm_event=armed, 
                                             stage_event=staged, 
                                             stop_event=stop, 
                                             staged_changes=staged_changes, 
                                             unrecognized_params=unrecognized_params)

    heartbeat_thread.start()
    #refparam_thread.start()
    update_params_thread.start()
    while True:
        staged_changes.put({'ASPD_SCALE_1': {'Value': 1.0, 'Type': 9}, 
                            'BAT1_A_PER_V': {'Value': 36.367515563964844, 'Type': 9},
                            'BAT1_CAPACITY': {'Value': -1.0, 'Type': 9},
                            'BAT1_I_CHANNEL': {'Value': -1, 'Type': 6},
                            'CAL_MAG0_ZSCALE': {'Value': 0.9313523173332214, 'Type': 9}, 
                            'CAL_MAG1_ID': {'Value': 396809, 'Type': 6}, 
                            'CAL_MAG1_PITCH': {'Value': 0.0, 'Type': 9}, 
                            'CAL_MAG1_PRIO': {'Value': 75, 'Type': 6},}) 
        """'BAT1_A_PER_V': {'Value': 36.367515563964844, 'Type': 9}, 
        'BAT1_CAPACITY': {'Value': -1.0, 'Type': 9}, 
        'BAT1_I_CHANNEL': {'Value': -1, 'Type': 6}})"""
        unrecognized_params.put({"BAT1_N_CELLS" : {"Value" : 6, "Type" : 6}})
        staged.set()
        time.sleep(2)
    #print(update_drone_params({"BAT1_N_CELLS" : {"Value" : 6, "Type" : 6}}, connection, connection_read_lock))

    """
    with open("drone_params.json", "w") as file:
        json.dump(drone_params, file)
    
    with open("test_params1.params", "r") as params_file:
        reference_params = params_file.read()
        reference_params = parse_params_file(reference_params)
    with open("drone_params1.json", "r") as params_file1:
        reference_params1 = json.load(params_file1)

    print(compare_param_list(reference_params, reference_params1))"""

if __name__ == '__main__':
    main_thread()    
    pass