import yaml

with open("../calibrationdata/ost.yaml", 'r') as f:
    try:
        doc = yaml.load(f)
    except yaml.YAMLError as exc:
        print(exc)

    txt = doc["distortion_coefficients"]["data"]
    print txt