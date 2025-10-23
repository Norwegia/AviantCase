from pymavlink import mavutil
import requests
import time
import struct
import logging
from typing import Dict, Any, Tuple, List, Optional

logger = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s - %(message)s',
    datefmt='%H:%M:%S'
)
PARAMS_URL = "https://caseparams.sandbox.aviant.no/reference.params"
MAVLINK_CONNECTION_URL = "/dev/tty.usbmodem01"
BAUD = 57600
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

REF_PARAMS_REFRESH_PERIOD = 1  # s
ACKNOWLEDGE_TIMEOUT = 2  # s


def get_reference_params() -> Optional[str]:
    """
    Fetch reference parameters from the remote URL.

    Makes an HTTP GET request to PARAMS_URL to retrieve the reference
    parameter file containing the latest parameter configurations.

    Returns:
        Optional[str]: The response text containing parameter data if successful,
                      None if the request fails or returns a non-200 status code.
    """
    response = requests.get(PARAMS_URL)
    if response.status_code == 200:
        return response.text


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


def refresh_reference_params() -> Dict[str, Dict[str, Any]]:
    """
    Fetch and parse the latest reference parameters from the remote source.

    Combines get_reference_params() and parse_params_file() to retrieve
    the most current parameter configuration and return it in a structured format.

    Returns:
        Dict[str, Dict[str, Any]]: Parsed parameter dictionary with parameter names
                                  as keys and dictionaries containing 'Value' and 'Type'
                                  as values. Returns empty dict if fetch fails.
    """
    text = get_reference_params()
    return parse_params_file(text)


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
            type="PARAM_VALUE", blocking=True, timeout=1)
        if message is None:
            break
        value = struct.unpack(MAV_PARAM_FORMAT_TYPE[message.param_type], struct.pack(
            'f', message.param_value))[0]  # typeprune from float
        params[message.param_id] = {
            "Value": value,
            "Type": message.param_type
        }

    return params


def compare_param_list(reference_params: Dict[str, Dict[str, Any]],
                       drone_params: Dict[str, Dict[str, Any]]) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, Dict[str, Any]]]:
    """
    Compare reference parameters against drone parameters to find differences.

    Identifies parameters that exist in both lists but have different values,
    and parameters that exist in the reference but not on the drone. Uses an
    optimization to track the last match index for efficient comparison.

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
            MAV_PARAM_FORMAT_TYPE[value["Type"]], value["Value"]))[0]  # typeprune to float
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            param.encode("utf-8"),
            packed_value,
            value["Type"]
        )
        unacknowlaged[param] = value

    start_time = time.perf_counter()
    while time.perf_counter() - start_time < ACKNOWLEDGE_TIMEOUT and unacknowlaged:
        message = connection.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=ACKNOWLEDGE_TIMEOUT)
        if message:
            if message.param_id.strip() in unacknowlaged:
                unacknowlaged.pop(message.param_id)
    return unacknowlaged


def create_qgc_sync_message(diff: Dict[str, Dict[str, Any]],
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
    connection = mavutil.mavlink_connection(MAVLINK_CONNECTION_URL, baud=BAUD)
    if not connection:
        logger.info(
            f"Failed to establish MAVLink connection at {MAVLINK_CONNECTION_URL}")
        return
    logger.info(f"Mavlink connection established at:{MAVLINK_CONNECTION_URL}")
    logger.info("Waiting for heartbeat...")
    heartbeat = connection.recv_match(
        type="HEARTBEAT", blocking=True, timeout=10)
    while not heartbeat:
        logger.info("Timed out waiting for heartbeat, trying again...")
        heartbeat = connection.recv_match(
            type="HEARTBEAT", blocking=True, timeout=10)
    logger.info("Got heartbeat")
    drone_params = get_params_on_drone(connection)
    reference_params = get_reference_params()
    reference_params = parse_params_file(reference_params)
    ref_param_update_time = time.perf_counter()
    armed = heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

    while True:
        if not armed:
            diff = {}
            if time.perf_counter() - ref_param_update_time > REF_PARAMS_REFRESH_PERIOD:
                reference_params = get_reference_params()
                reference_params = parse_params_file(reference_params)
                ref_param_update_time = time.perf_counter()
                diff, unrecognized_params = compare_param_list(
                    reference_params, drone_params)
                if diff:
                    unacknowledged_params = update_drone_params(
                        diff, connection)
                    message = create_qgc_sync_message(
                        diff, unrecognized_params, unacknowledged_params)
                    logger.info(message)
                    for param, value in diff.items():
                        if param not in unacknowledged_params and param not in unrecognized_params:
                            drone_params[param] = value
                    connection.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_NOTICE, message.encode("utf-8"))

        heartbeat = connection.recv_match(
            type="HEARTBEAT", blocking=True, timeout=1)
        if heartbeat:
            logger.info("Got heartbeat")
            armed = heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


if __name__ == '__main__':
    main()
