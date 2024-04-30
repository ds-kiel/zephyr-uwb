
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


dev_positions = {

}


factory_delays = {
}

SLOTS_PER_PHASE = 7


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

                if orig_msg["event"] == "rx":
                    msg = {
                        'event': orig_msg['event'],
                        #'own_number': orig_msg['own_id'],
                        'own_number': orig_msg['other_id'],
                        #'rx_number': orig_msg['other_id'],
                        'rx_number': orig_msg['own_id'],
                        'rx_round': orig_msg['rtc_round_ts'],
                        #'rx_phase': orig_msg['phase'],
                        'rx_slot': orig_msg['slot'] + (int(orig_msg['phase'])-1) * SLOTS_PER_PHASE, # TODO: this is a bit hacky, no?!
                        'rx_ts': orig_msg['ts'],
                        'bias_corrected_rx_ts': orig_msg.get('ts_bias_corrected', orig_msg['ts'])
                    }
                elif orig_msg["event"] == "tx":
                    msg = {
                        'event': orig_msg['event'],
                        'own_number': orig_msg['own_id'],
                        'tx_round': orig_msg['rtc_round_ts'],
                        # 'rx_phase': orig_msg['phase'],
                        'tx_slot': orig_msg['slot'] + (int(orig_msg['phase']) - 1) * SLOTS_PER_PHASE,
                        # TODO: this is a bit hacky, no?!
                        'tx_ts': orig_msg['ts']
                    }
                else:
                    print("ignoring", orig_msg)
                    continue

                print(msg)

                msg['_log_ts'] = 0
                dev = devs[orig_msg['own_id']]
                yield (msg['_log_ts'], dev, msg)
            except json.decoder.JSONDecodeError:
                print(json_str)
                pass
        except ValueError:
            print("ValueError")
            pass


import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.offsetbox import (OffsetImage, AnnotationBbox)

import numpy as np
from PIL import Image


def draw_layout(plt):
    lineargs = {
        "color": "black",
        #"alpha": 0.5,
        "lw": 1.5,
        "zorder": -1,
    }

    rectargs = {
        "facecolor": "white",
        "edgecolor": "black",
        #"alpha": 0.5,
        "lw": 1.5,
        "zorder": -1,
    }

    pillar_size = 0.25
    # npimage = np.flip(np.asarray(Image.open('img/lille.png')), axis=0)
    # scalingx = 0.0325
    # scalingy = 0.027
    # tx = dev_positions['dwm1001-1'][0]-0.02
    # ty = dev_positions['dwm1001-1'][1]-0.02

    # the axes might be wrong here!
    # plt.gca().imshow(npimage, origin="upper", extent=(tx, tx + npimage.shape[0]*scalingx, ty, ty + npimage.shape[1]*scalingy), zorder=-1)

    plt.plot([dev_positions['dwm1001-1'][0], dev_positions['dwm1001-1'][0]], [22, 32], **lineargs)
    plt.plot([dev_positions['dwm1001-12'][0], dev_positions['dwm1001-12'][0]], [22, dev_positions['dwm1001-14'][1]+0.25], **lineargs)
    plt.plot([dev_positions['dwm1001-14'][0], 12.0], [dev_positions['dwm1001-14'][1]+0.25, dev_positions['dwm1001-14'][1]+0.25], **lineargs)

    plt.gca().add_patch(Rectangle((dev_positions['dwm1001-1'][0]-pillar_size*0.5, dev_positions['dwm1001-1'][1]-pillar_size), pillar_size, pillar_size, **rectargs))
    plt.gca().add_patch(Rectangle((0.375-pillar_size, 28.437-pillar_size), pillar_size, pillar_size, **rectargs))
    plt.gca().add_patch(Rectangle((5.50, 28.17), pillar_size, pillar_size, **rectargs))
    plt.gca().add_patch(Rectangle((5.50, dev_positions['dwm1001-1'][1]-pillar_size), pillar_size, pillar_size, **rectargs))


