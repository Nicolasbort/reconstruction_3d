#! /usr/bin/env python3

import yaml

def getInitialPose(file_path):
    with open(file_path, "r") as stream:
        loaded_data = yaml.safe_load(stream)

    print("Data loaded: {} \n".format(loaded_data["start_position"]))

    x = int(loaded_data["start_position"]["x"])
    y = int(loaded_data["start_position"]["y"])
    z = int(loaded_data["start_position"]["z"])
    heading = int(loaded_data["start_position"]["heading"])

    return [x, y, z, heading]



def loadYaml(file_path):
    with open(file_path, "r") as stream:
        loaded_data = yaml.safe_load(stream)

    return loaded_data



def getObjectPose(file_path):
    with open(file_path, "r") as stream:
        loaded_data = yaml.safe_load(stream)

    print("Data loaded: {} \n".format(loaded_data["start_position"]))

    x = int(loaded_data["start_position"]["x"])
    y = int(loaded_data["start_position"]["y"])
    z = int(loaded_data["start_position"]["z"])

    return [x, y, z]