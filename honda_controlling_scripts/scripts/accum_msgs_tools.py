#!/usr/bin/env python

"""
Tools to filter messages from accumulated
read with the panda in blocks.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


def to_sendeable_block(array_of_msgs, can_bus=0):
    """Given an array of msgs return it in the format
    panda.can_send_many expects.

    Input looks like: [(512, bytearray(b'>\x80\x1c'), 0), ...]
    Output looks like: [(512, '>\x80\x1c', 0), ...]
    """
    new_arr = []
    for msg in array_of_msgs:
        new_arr.append((msg[0], msg[1], str(msg[2]), can_bus))
    return new_arr


def to_sendeable_block_original_bus(array_of_msgs):
    """Given an array of msgs return it in the format
    panda.can_send_many expects.

    Input looks like: [(512, bytearray(b'>\x80\x1c'), 0), ...]
    Output looks like: [(512, '>\x80\x1c', 0), ...]
    """
    new_arr = []
    for msg in array_of_msgs:
        new_arr.append((msg[0], msg[1], str(msg[2]), msg[3]))
    return new_arr


def to_sendeable_accum(accum, can_bus=0):
    """Given an array of array of msgs (so array of panda.can_recv)
    return it in the format
    panda.can_send_many expects.

    Input looks like: [[(512, bytearray(b'>\x80\x1c'), 0), ...],[...]]
    Output looks like: [[(512, '>\x80\x1c', 0), ...], [...]]
    """
    new_acc = []
    for arr in accum:
        new_acc.append(to_sendeable_block(arr))
    return new_acc


def to_sendeable_accum_original_bus(accum):
    """Given an array of array of msgs (so array of panda.can_recv)
    return it in the format
    panda.can_send_many expects.

    Input looks like: [[(512, bytearray(b'>\x80\x1c'), 0), ...],[...]]
    Output looks like: [[(512, '>\x80\x1c', 0), ...], [...]]
    """
    new_acc = []
    for arr in accum:
        new_acc.append(to_sendeable_block_original_bus(arr))
    return new_acc


def filter_messages_remove(frame_ids_to_remove, array_of_msgs):
    """Given an array of can msgs, remove the messages that have the ids
    in frame_ids_to_remove and return. If the data field is in bytearray
    format it will be changed to str.
    """
    filtered_arr = []
    for msg in array_of_msgs:
        frame_id, _, data, bus = msg
        if type(data) != str:
            msg = (frame_id, _, str(data), bus)
        remove_frame = False
        for id_to_remove in frame_ids_to_remove:
            if frame_id == id_to_remove:
                remove_frame = True
                break
        if not remove_frame:
            filtered_arr.append(msg)
    return filtered_arr


def filter_messages_keep(frame_ids_to_keep, array_of_msgs):
    """Given an array of can msgs, remove the messages that have  ids
    NOT in frame_ids_to_keep and return. If the data field is in bytearray
    format it will be changed to str.
    """
    filtered_arr = []
    for msg in array_of_msgs:
        frame_id, _, data, bus = msg
        if type(data) != str:
            msg = (frame_id, _, str(data), bus)
        for id_to_keep in frame_ids_to_keep:
            if frame_id == id_to_keep:
                filtered_arr.append(msg)
    return filtered_arr


def filter_messages_keep_bus(frame_ids_to_keep, array_of_msgs, bus_to_keep):
    """Given an array of can msgs, remove the messages that have  ids
    NOT in frame_ids_to_keep and return. If the data field is in bytearray
    format it will be changed to str.
    """
    filtered_arr = []
    for msg in array_of_msgs:
        frame_id, _, data, bus = msg
        if type(data) != str:
            msg = (frame_id, _, str(data), bus)
        for id_to_keep in frame_ids_to_keep:
            if frame_id == id_to_keep and bus == bus_to_keep:
                filtered_arr.append(msg)
    return filtered_arr


def filter_accum_remove(frame_ids_to_remove, accum, timestamps):
    """Do filter_message_remove for every block in the array
    of can_recv.
    """
    filtered_accum = []
    filtered_timestamps = []
    for array_of_msgs, timestamp in zip(accum, timestamps):
        filtered_array = filter_messages_remove(
            frame_ids_to_remove, array_of_msgs)
        if filtered_array:
            filtered_timestamps.append(timestamp)
            filtered_accum.append(filtered_array)
    return filtered_accum, filtered_timestamps


def filter_accum_keep(frame_ids_to_keep, accum, timestamps):
    """Do filter_messages_keep for every block in the array of
    can_recv.
    """
    filtered_accum = []
    filtered_timestamps = []
    for array_of_msgs, timestamp in zip(accum, timestamps):
        filtered_array = filter_messages_keep(frame_ids_to_keep, array_of_msgs)
        if filtered_array:
            filtered_timestamps.append(timestamp)
            filtered_accum.append(filtered_array)
    return filtered_accum, filtered_timestamps


def filter_accum_keep_bus(frame_ids_to_keep, accum, timestamps, bus):
    """Do filter_messages_keep for every block in the array of
    can_recv.
    """
    filtered_accum = []
    filtered_timestamps = []
    for array_of_msgs, timestamp in zip(accum, timestamps):
        filtered_array = filter_messages_keep_bus(frame_ids_to_keep, array_of_msgs, bus)
        if filtered_array:
            filtered_timestamps.append(timestamp)
            filtered_accum.append(filtered_array)
    return filtered_accum, filtered_timestamps


def replace_message_by_id(message_id, array_of_msgs, new_message_data):
    filtered_arr = []
    for msg in array_of_msgs:
        frame_id, _, data, bus = msg
        if message_id == frame_id:
            msg = (frame_id, _, str(new_message_data), bus)
        filtered_arr.append(msg)
    return filtered_arr


def replace_messages_by_id(message_id, accum, new_message_data):
    filtered_accum = []
    for array_of_msgs in accum:
        filtered_array = replace_message_by_id(
            message_id, array_of_msgs, new_message_data)
        filtered_accum.append(filtered_array)
    return filtered_accum
