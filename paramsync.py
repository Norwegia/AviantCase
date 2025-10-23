import requests
import time
import struct
import logging
import serial
from pymavlink import mavutil
from typing import Dict, Any, Tuple, List, Optional

logger = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)

"""Global Params"""

PARAMS_URL = "https://caseparams.sandbox.aviant.no/reference.params"
MAVLINK_CONNECTION_URLS = [
    "/dev/tty.usbmodem01",  # OSX USB
    # "/dev/ttyAMA0",  # Linux Serial
    # "udpin:localhost:14540" # SITL
]
BAUD = 57600
REBOOT_ON_SYNC = False  # should the vehicle reboot when new parameters are synced
REF_PARAMS_REFRESH_PERIOD = 1  # s
REF_PARAMS_GET_TIMEOUT = 2  # s
PARAM_ACKNOWLEDGE_TIMEOUT = 2  # s
MAVLINK_LOSS_TIMEOUT = 2  # s

MAV_PARAM_TYPE = {
    1: int,
    2: int,
    3: int,
    4: int,
    5: int,
    6: int,
    7: int,
    8: int,
    9: float,
    10: float,
}

MAV_PARAM_FORMAT_TYPE = {
    1: 'B',
    2: 'b',
    3: 'H',
    4: 'h',
    5: 'I',
    6: 'i',
    7: 'Q',
    8: 'q',
    9: 'f',
    10: 'd',
}


def parse_params_file(file: str) -> Dict[str, Dict[str, Any]]:
    """
    Parse a parameter file string into a structured dictionary.

    Takes a parameter file content (typically from a .params file) and converts
    it into a dictionary format. The function skips header lines starting with '#'
    and extracts parameter names, values, and types from tab-separated columns.

    Args:
        file (str): The parameter file content as a string with tab-separated values.
                   Expected format: lines with vehicle_id, component_id, param_name, 
                   param_value, param_type columns.

    Returns:
        Dict[str, Dict[str, Any]]: Dictionary where keys are parameter names and 
                                  values are dictionaries containing:
                                  - 'Value': The parameter value converted to proper type
                                  - 'Type': The MAVLink parameter type as integer
    """
    lines = file.strip().split('\n')
    slice = 0
    while True:  # find index of first line that is not a header
        if lines[slice][0] != "#":
            break
        slice += 1
    # ignore vehicle & component id
    values = [line.split('\t')[2:] for line in lines[slice:]]
    key_value_pairs = {}
    for value in values:
        key_value_pairs[value[0]] = {
            "Value": MAV_PARAM_TYPE[int(value[2])](value[1]),
            "Type": int(value[2])
        }  # convert values to proper data type
    return key_value_pairs


def get_reference_params(connection: mavutil.mavlink_connection) -> Optional[str]:
    """
    Fetch reference parameters from the remote URL.

    Makes an HTTP GET request to PARAMS_URL to retrieve the reference
    parameter file containing the latest parameter configurations. 
    Notifies QGroundControl & logs if request fails.

    Returns:
        Optional[str]: A JSON formatted dict of the parameter data if successful,
                      None if the request fails or returns a non-200 status code.
    """
    try:
        response = requests.get(PARAMS_URL, timeout=REF_PARAMS_GET_TIMEOUT)
        if response.status_code == 200:
            return parse_params_file(response.text)
        else:
            message = f"Failed to fetch reference parameters: {response.status_code}"
    except (requests.exceptions.ConnectionError,
            requests.exceptions.ConnectTimeout,
            requests.exceptions.ReadTimeout,
            requests.exceptions.RequestException):
        message = f"GET request to reference parameters timed out"
    logger.error(message)
    connection.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_NOTICE, message.encode("utf-8"))


def get_params_on_drone(connection: mavutil.mavlink_connection) -> Dict[str, Dict[str, Any]]:
    """
    Retrieve all parameters currently stored on the drone.

    Sends a PARAM_REQUEST_LIST message to the drone and collects all PARAM_VALUE
    responses to build a complete parameter dictionary. Values are converted from
    float format to their proper data types based on the parameter type.

    Args:
        connection (mavutil.mavlink_connection): Active MAVLink connection to the drone.

    Returns:
        Dict[str, Dict[str, Any]]: Dictionary where keys are parameter names and
                                  values are dictionaries containing:
                                  - 'Value': The parameter value in proper data type
                                  - 'Type': The MAVLink parameter type as integer
    """
    connection.mav.param_request_list_send(
        connection.target_system,
        connection.target_component
    )
    params = {}
    while True:
        message = connection.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=PARAM_ACKNOWLEDGE_TIMEOUT)
        if message is None:
            break
        value = struct.unpack(MAV_PARAM_FORMAT_TYPE[message.param_type], struct.pack(
            'f', message.param_value))[0]  # reinterpret float as proper data type
        params[message.param_id] = {
            "Value": value,
            "Type": message.param_type
        }

    return params


def compare_param_lists(reference_params: Dict[str, Dict[str, Any]],
                        drone_params: Dict[str, Dict[str, Any]]) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, Dict[str, Any]]]:
    """
    Compare reference parameters against drone parameters to find differences.

    Identifies parameters that exist in both lists but have different values,
    and parameters that exist in the reference but not on the drone. Expects 
    both lists to be sorted alphabetically.

    Args:
        reference_params (Dict[str, Dict[str, Any]]): Reference parameter dictionary
                                                     with 'Value' and 'Type' keys.
        drone_params (Dict[str, Dict[str, Any]]): Current drone parameter dictionary
                                                 with 'Value' and 'Type' keys.

    Returns:
        Tuple[Dict[str, Dict[str, Any]], Dict[str, Dict[str, Any]]]: A tuple containing:
            - diff: Parameters that exist on drone but have different values
            - unrecognized_params: Parameters in reference but not found on drone
    """
    last_match_idx = 0
    diff = {}
    unrecognized_params = {}
    found_match = False
    for ref_param, ref_value in reference_params.items():
        for idx, drone_param in enumerate(list(drone_params.keys())[last_match_idx:]):
            if ref_param == drone_param:
                last_match_idx = idx + 1  # make use of alphabetic sorting to optimize parsing
                drone_value = drone_params[drone_param]
                if ref_value != drone_value:
                    diff[ref_param] = ref_value
                found_match = True
                break
        if not found_match:
            unrecognized_params[ref_param] = ref_value
        found_match = False
    return diff, unrecognized_params


def update_drone_params(params: Dict[str, Dict[str, Any]], connection: mavutil.mavlink_connection) -> List[str]:
    """
    Send parameter updates to the drone and verify acknowledgment.

    Iterates through the provided parameters and sends PARAM_SET messages to update
    each parameter on the drone. Values are converted to float format as required
    by the MAVLink protocol. Waits for PARAM_VALUE acknowledgment messages.

    Args:
        params (Dict[str, Dict[str, Any]]): Parameters to update with 'Value' and 'Type'.
        connection (mavutil.mavlink_connection): Active MAVLink connection to the drone.

    Returns:
        List[str]: List of parameter names that were not acknowledged by the drone
                   within the timeout period.
    """
    unacknowlaged = {}
    for param, value in params.items():
        packed_value = struct.unpack('f', struct.pack(
            MAV_PARAM_FORMAT_TYPE[value["Type"]], value["Value"]))[0]  # reinterpret as float
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            param.encode("utf-8"),
            packed_value,
            value["Type"]
        )
        unacknowlaged[param] = value

    start_time = time.perf_counter()
    while unacknowlaged and time.perf_counter() - start_time < PARAM_ACKNOWLEDGE_TIMEOUT:
        message = connection.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=PARAM_ACKNOWLEDGE_TIMEOUT)
        if message:
            if message.param_id.strip() in unacknowlaged:
                unacknowlaged.pop(message.param_id)
    return unacknowlaged


def create_log_message(diff: Dict[str, Dict[str, Any]],
                       unrecognized_params: Dict[str, Dict[str, Any]], unsynced_params: Dict[str, Dict[str, Any]]) -> str:
    """
    Generate a human-readable status message for QGroundControl display.

    Creates a formatted text message summarizing the parameter synchronization
    results, including successfully updated parameters, parameters not found on
    the drone, and parameters that failed to sync.

    Args:
        diff (Dict[str, Dict[str, Any]]): Parameters that were updated with new values.
        unrecognized_params (Dict[str, Dict[str, Any]]): Parameters not found on drone.
        unsynced_params (Dict[str, Dict[str, Any]]): Parameters that failed to sync.

    Returns:
        str: Formatted message suitable for display in QGroundControl status text,
             containing sections for updated, unrecognized, and unsynced parameters.
    """
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
    if not REBOOT_ON_SYNC:
        message += "\nReboot vehicle for parameter changes to take effect."
    return message


def main() -> None:
    """
    Single-threaded main loop for parameter synchronization.

    Establishes MAVLink connection, waits for heartbeat, and continuously monitors
    for parameter differences between reference and drone. Updates drone parameters
    when differences are found and the drone is disarmed. Sends status messages
    to QGroundControl.

    This is a simpler, single-threaded implementation that periodically refreshes
    both reference parameters and drone parameters, then synchronizes any differences.
    """
    while True:
        connected = False
        for url in MAVLINK_CONNECTION_URLS:
            try:
                connection = mavutil.mavlink_connection(url, baud=BAUD)
                connected = True
            except serial.serialutil.SerialException:
                logger.error(f"Could not open {url}")
        if not connected:
            logger.error(
                f"Failed to establish MAVLink connection with any of {MAVLINK_CONNECTION_URLS}. Trying again...")
            time.sleep(MAVLINK_LOSS_TIMEOUT)
            continue
        logger.info(f"MAVlink connection established at: {url}")
        logger.info("Waiting for heartbeat...")
        try:
            heartbeat = connection.recv_match(
                type="HEARTBEAT", blocking=True, timeout=10)
            while not heartbeat:
                logger.info("Timed out waiting for heartbeat, trying again...")
                heartbeat = connection.recv_match(
                    type="HEARTBEAT", blocking=True, timeout=10)
        except serial.serialutil.SerialException:
            logger.error(
                "Error reading heartbeat. Retrying MAVLink connection...")
            time.sleep(MAVLINK_LOSS_TIMEOUT)
            continue
        logger.info("Got heartbeat")

        drone_params = get_params_on_drone(connection)
        reference_params = None
        while reference_params is None:  # check for internet connection
            reference_params = get_reference_params(connection)
        ref_param_last_update = time.perf_counter()
        armed = heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

        while connected:
            try:
                if not armed:
                    diff = {}
                    if time.perf_counter() - ref_param_last_update > REF_PARAMS_REFRESH_PERIOD:
                        reference_params = get_reference_params(connection)
                        ref_param_last_update = time.perf_counter()
                        if reference_params is None:  # failed to fetch from website
                            continue
                        diff, unrecognized_params = compare_param_lists(
                            reference_params, drone_params)
                        if diff:
                            unacknowledged_params = update_drone_params(
                                diff, connection)
                            message = create_log_message(
                                diff, unrecognized_params, unacknowledged_params)
                            for param, value in diff.items():
                                if param not in unacknowledged_params and param not in unrecognized_params:
                                    drone_params[param] = value
                            logger.info(message)
                            connection.mav.statustext_send(
                                mavutil.mavlink.MAV_SEVERITY_NOTICE, b"Synced onboard params to online reference")
                            if REBOOT_ON_SYNC:
                                connection.reboot_autopilot()

                heartbeat = connection.recv_match(
                    type="HEARTBEAT", blocking=True, timeout=1)
                if heartbeat:
                    logger.info("Got heartbeat")
                    armed = heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            except serial.serialutil.SerialException:  # handle loss of MAVLink connection during main loop gracefully
                logger.error(
                    "Error reading from MAVLink connection: {url}. Attempting to restablish connection...")
                connected = False


if __name__ == '__main__':
    main()
