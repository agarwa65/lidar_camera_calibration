#!/usr/bin/env python

import yaml

# with open("ost.yaml", 'r') as stream:
#     try:
#         print(yaml.load(stream))
#     except yaml.YAMLError as exc:
#         print(exc)


with open("calibrationdata/ost.yaml", 'r') as f:
    try:
        doc = yaml.load(f)
    except yaml.YAMLError as exc:
        print(exc)

    txt = doc["camera_name"]
    print txt