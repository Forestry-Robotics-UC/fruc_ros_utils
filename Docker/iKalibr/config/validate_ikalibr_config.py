#!/usr/bin/env python3
import sys
import os
try:
    import yaml
except Exception as e:
    print('MISSING_PYYAML')
    sys.exit(2)

path = os.path.join(os.path.dirname(__file__), 'ikalibr-config.yaml')
if not os.path.exists(path):
    print('MISSING_FILE')
    sys.exit(2)

with open(path,'r') as f:
    data = yaml.safe_load(f)

errs = []
# top-level
if 'Configor' not in data:
    errs.append('Missing top-level: Configor')
else:
    conf = data['Configor']
    ds = conf.get('DataStream')
    if ds is None:
        errs.append('Missing: DataStream')
    else:
        if 'IMUTopics' not in ds:
            errs.append('Missing: IMUTopics')
        if 'CameraTopics' not in ds:
            errs.append('Missing: CameraTopics')
        if 'RGBDTopics' not in ds:
            errs.append('Missing: RGBDTopics')
    pref = conf.get('Preference')
    if pref is None:
        errs.append('Missing: Preference')
    else:
        if 'SplineScaleInViewer' not in pref:
            errs.append('Missing: Preference.SplineScaleInViewer')
        else:
            try:
                if float(pref['SplineScaleInViewer']) <= 0.0:
                    errs.append('Preference.SplineScaleInViewer must be > 0')
            except Exception:
                errs.append('Preference.SplineScaleInViewer not a number')
        if 'CoordSScaleInViewer' not in pref:
            errs.append('Missing: Preference.CoordSScaleInViewer')
        else:
            try:
                if float(pref['CoordSScaleInViewer']) <= 0.0:
                    errs.append('Preference.CoordSScaleInViewer must be > 0')
            except Exception:
                errs.append('Preference.CoordSScaleInViewer not a number')

if errs:
    print('INVALID')
    for e in errs:
        print('- ' + e)
    sys.exit(1)
else:
    print('OK')
    sys.exit(0)
