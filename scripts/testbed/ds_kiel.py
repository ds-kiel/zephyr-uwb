from PIL import Image
import numpy as np
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import matplotlib as mpl


name = 'ds_kiel'

devs = [
    'profiled-gateway-2-dwm1001-000760119483',
    'profiled-gateway-1',
    'profiled-gateway-2-dwm1001-000760120271',
    'gateway-503-1',
    'mobile-gateway',
    'gateway-503-0',
    'gateway-503-2',
]

ev_positions = {}


factory_delays = {}

SLOTS_PER_PHASE = 4


def get_dist(da, db):
    if sorted([da, db]) == sorted(['gateway-503-1', 'gateway-503-0']):
        return 4.7

    if sorted([da, db]) == sorted(['gateway-503-2', 'gateway-503-1']):
        return 4.6

    if sorted([da, db]) == sorted(['gateway-503-2', 'gateway-503-0']):
        return 5.3

    if sorted([da, db]) == sorted(['gateway-503-0', 'gateway-503-4']):
        return 1.3
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
                        'own_number': orig_msg['ranging-id'],
                        # 'own_number': orig_msg['other_id'],
                        'rx_number': orig_msg['other-id'],
                        # 'rx_number': orig_msg['own_id'],
                        'rx_round': orig_msg['asn'],
                        # 'rx_phase': orig_msg['phase'],
                        'rx_slot': orig_msg['slot']
                        + int(orig_msg['phase'])
                        * SLOTS_PER_PHASE,  # TODO: this is a bit hacky, no?!
                        'rx_ts': orig_msg['timestamp'],
                        'fp_index': orig_msg['fp-index'],
                        'fp_ampl1': orig_msg['fp-ampl1'],
                        'fp_ampl2': orig_msg['fp-ampl2'],
                        'fp_ampl3': orig_msg['fp-ampl3'],
                        'cir_pwr': orig_msg['cir-pwr'],
                        'cfo': orig_msg['cfo'],
                        'rx_pacc': orig_msg['rx-pacc'],
                        'bias_corrected_rx_ts': orig_msg.get(
                            'ts_bias_corrected', orig_msg['timestamp']
                        ),
                    }
                elif orig_msg['event'] == 'tx':
                    msg = {
                        'event': orig_msg['event'],
                        'own_number': orig_msg['ranging-id'],
                        'tx_round': orig_msg['asn'],
                        # 'rx_phase': orig_msg['phase'],
                        # TODO: this is a bit hacky, no?!
                        'tx_slot': orig_msg['slot']
                        + int(orig_msg['phase']) * SLOTS_PER_PHASE,
                        'tx_ts': orig_msg['timestamp'],
                    }
                else:
                    print('ignoring', orig_msg)
                    continue

                print(msg)

                msg['_log_ts'] = 0
                dev = devs[orig_msg['ranging-id']]
                yield (msg['_log_ts'], dev, msg)
            except json.decoder.JSONDecodeError:
                print(json_str)
                pass
        except ValueError:
            print('ValueError')
            pass
