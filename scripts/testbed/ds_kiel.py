from PIL import Image
import numpy as np
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import matplotlib as mpl


name = 'ds_kiel'

devs = [
    'gateway-stativ-2',
    'gateway-stativ-1',
    'gateway-stativ-3',
    'gateway-stativ-4',
    'iot-gateway-9',
    'iot-gateway-0',
    'iot-gateway-1',
    'iot-gateway-2',
    'iot-gateway-6',
    'iot-gateway-8',
    'profiled-gateway-1',
    'mobile-gateway',
]

dev_positions = {}

factory_delays = {}

SLOTS_PER_PHASE = 4


def get_dist(da, db):
    if sorted([da, db]) == sorted(['gateway-stativ-1', 'gateway-stativ-2']):
        return 4.201

    if sorted([da, db]) == sorted(['gateway-stativ-1', 'gateway-stativ-3']):
        return 4.967

    if sorted([da, db]) == sorted(['gateway-stativ-1', 'gateway-stativ-4']):
        return 3.565

    if sorted([da, db]) == sorted(['gateway-stativ-2', 'gateway-stativ-3']):
        return 2.456

    if sorted([da, db]) == sorted(['gateway-stativ-2', 'gateway-stativ-4']):
        return 4.315

    if sorted([da, db]) == sorted(['gateway-stativ-3', 'gateway-stativ-4']):
        return 3.975

    return 0.0


def parse_messages_from_lines(line_it, src_dev=None):
    import json

    dev_set = set(devs)

    if src_dev is not None:
        dev_set = {src_dev}

    for line in line_it:
        if line.strip() == "":
            continue
        try:
            # {"event": "rx", "own_id": 1, "other_id": 5, "rtc_round_ts": 11259612, "phase": 1, "slot": 1, "ts": 25625173051}
            json_str = line

            try:
                orig_msg = json.loads(json_str)

                if orig_msg['event'] == 'rx':
                    msg = {
                        'event': orig_msg['event'],
                        'own_number': orig_msg['own_id'],
                        # 'own_number': orig_msg['other_id'],
                        'rx_number': orig_msg['other_id'],
                        # 'rx_number': orig_msg['own_id'],
                        'rx_round': orig_msg['asn'],
                        # 'rx_phase': orig_msg['phase'],
                        'rx_slot': orig_msg['slot']
                        + int(orig_msg['phase'])
                        * SLOTS_PER_PHASE,  # TODO: this is a bit hacky, no?!
                        'rx_ts': orig_msg['ts'],
                        'fp_index': orig_msg['fp_index'],
                        'fp_ampl1': orig_msg['fp_ampl1'],
                        'fp_ampl2': orig_msg['fp_ampl2'],
                        'fp_ampl3': orig_msg['fp_ampl3'],
                        'cir_pwr': orig_msg['cir_pwr'],
                        'ci': orig_msg['cfo'] / (-0.000573121584378756 * 1e6),
                        'rx_pacc': orig_msg['rx_pacc'],
                        'bias_corrected_rx_ts': orig_msg.get(
                            'ts_bias_corrected', orig_msg['ts']
                        ),
                    }
                elif orig_msg['event'] == 'tx':
                    msg = {
                        'event': orig_msg['event'],
                        'own_number': orig_msg['own_id'],
                        'tx_round': orig_msg['asn'],
                        # 'rx_phase': orig_msg['phase'],
                        # TODO: this is a bit hacky, no?!
                        'tx_slot': orig_msg['slot']
                        + int(orig_msg['phase']) * SLOTS_PER_PHASE,
                        'tx_ts': orig_msg['ts'],
                    }
                else:
                    print('ignoring', orig_msg)
                    continue

                print(msg)

                msg['_log_ts'] = 0
                dev = devs[orig_msg['own_id']]
                yield (msg['_log_ts'], dev, msg)
            except json.decoder.JSONDecodeError:
                print(json_str)
                pass
        except ValueError:
            print('ValueError')
            pass
