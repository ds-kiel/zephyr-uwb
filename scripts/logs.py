import numpy as np

from base import get_dist, pair_index, convert_ts_to_sec, convert_sec_to_ts, convert_ts_to_m, convert_m_to_ts, ci_to_rd, rd_to_ci, convert_sec_to_m
DS_OVERFLOW_VAL = 0xFFFFFFFFFF+1


import pandas as pd
import io

def gen_messages_from_testbed_run(testbed, log_filepath):
    if log_filepath.endswith('.tar.gz'):
        import tarfile
        tar = tarfile.open(log_filepath, "r:gz")

        with tar:
            for member in tar.getmembers():
                if member.name.endswith('.log'):
                    logfile = tar.extractfile(member)
                    with io.TextIOWrapper(logfile) as f:
                        yield from testbed.parse_messages_from_lines(f)
    else:
        # TODO: Support gzip open as well?
        with open(log_filepath) as f:
            yield from testbed.parse_messages_from_lines(f)


def gen_round_events(testbed, run, iter_per_device=True):
    def gen_round_events_for_iter(it, filter_dev=None):
        it_rx_events = []
        it_tx_events = []
        it_round = None

        for log_ts, dev, x in it:
            if filter_dev is not None and filter_dev != dev:
                continue

            e = x['event']

            if e == 'rx':
                e_r = int(x['rx_round'])
            elif e == 'tx':
                e_r = int(x['tx_round'])
            else:
                continue

            if it_round is None:
                it_round = e_r

            if it_round < e_r:
                yield it_round, it_rx_events, it_tx_events
                it_rx_events = []
                it_tx_events = []
                it_round = e_r

            if e == 'rx':
                it_rx_events.append(x)
            elif e == 'tx':
                it_tx_events.append(x)
            elif e == 'init':
                pass
            else:
                print("Unknown event", x)

        if len(it_rx_events) > 0 or len(it_tx_events) > 0:
            yield it_round, it_rx_events, it_tx_events


    msg_gen = gen_messages_from_testbed_run(testbed, run)

    if iter_per_device:
        import itertools
        it_copies = itertools.tee(msg_gen, len(testbed.devs))
        dev_iters = [gen_round_events_for_iter(it, filter_dev=d) for d, it in zip(testbed.devs, it_copies)]

        round_events = [next(di, (None, [], [])) for di in dev_iters]
        while True:
            min_round = min([re[0] for re in round_events if re[0] is not None], default=None)

            if min_round is None:
                break  # we are done

            all_rx_events = []
            all_tx_events = []

            for i, re in enumerate(round_events):
                if re[0] == min_round:
                    all_rx_events += re[1]
                    all_tx_events += re[2]
                    round_events[i] = next(dev_iters[i], (None, [], [])) #advance this it
                else:
                    print("min_round", min_round, i, re[0])

            yield min_round, all_rx_events, all_tx_events
    else:
        yield from gen_round_events_for_iter(msg_gen)

def gen_all_rx_events(testbed, run, skip_to_round=None, until_round=None):
    round_gen = gen_round_events(testbed, run)

    for (r, rx_events, tx_events) in round_gen:
        if skip_to_round is not None and r < skip_to_round:
            continue
        if until_round is not None and r > until_round:
            break

        for rx_event in rx_events:
            yield rx_event

def extract_rx_tx_pairs(rx_df, tx_df, transmitter, receiver, bias_corrected=True):

    rel_rx = rx_df[(rx_df['own_number'] == receiver) & (rx_df['rx_number'] == transmitter)]

    last_rx_ts = None
    last_tx_ts = None

    for index, row in rel_rx.iterrows():

        poss_txs = tx_df[(tx_df['own_number'] == transmitter) & (tx_df['tx_round'] == row['rx_round']) & (tx_df['tx_slot'] == row['rx_slot'])]

        if len(poss_txs.index) >= 1:
            rel_tx = poss_txs.iloc[0]
        else:
            continue    # we could not find the relevant transmission!

        rx_ts = row['bias_corrected_rx_ts' if bias_corrected else 'rx_ts']
        tx_ts = rel_tx['tx_ts']

        # correct for overflowing timestamps
        if last_rx_ts is not None and rx_ts < last_rx_ts:
            rx_ts += DS_OVERFLOW_VAL

        # print(transmitter, receiver, row, rel_tx)
        # if last_rx_ts and last_tx_ts:
        #     print(rx_ts, last_rx_ts, convert_ts_to_sec(rx_ts - last_rx_ts), convert_ts_to_sec(tx_ts - last_tx_ts))
        #     exit()

        last_rx_ts = rx_ts

        if last_tx_ts is not None and tx_ts < last_tx_ts:
            tx_ts += DS_OVERFLOW_VAL
        last_tx_ts = tx_ts


        d = {
            'tx_number': rel_tx['own_number'],
            'rx_number': row['own_number'],
            'round': row['rx_round'],
            'slot': row['rx_slot'],
            'rx_ts': rx_ts,
            'tx_ts': tx_ts,
            'rx_ci': row['ci']
        }

        yield d

import math
import scipy
from scipy import stats

# alpha=0.01 corresponds to 99% confidence interval
def calc_ci_of_sd(sd, num, alpha=0.01):
    low = np.sqrt(((num-1)*(sd**2))/scipy.stats.chi2.ppf(1.0-alpha/2.0, num - 1))
    up = np.sqrt(((num-1)*(sd**2))/scipy.stats.chi2.ppf(alpha/2.0, num - 1))
    return (low, up)


def extract_data_slots(rx_df, tx_df, r, a, b, tdoa_src_dev_number, init_slot, response_slot, final_slot):
    # we extract data for initiator a and responder b (numbers not device names!)
    def first_or_none(l):
        return next(iter(l), None)

    init_tx = first_or_none(
        tx_df[(tx_df["tx_round"] == r) & (tx_df["tx_slot"] == init_slot) & (tx_df["own_number"] == a)].to_dict(
            orient='records'))
    init_rx = first_or_none(
        rx_df[(rx_df["rx_round"] == r) & (rx_df["rx_slot"] == init_slot) & (rx_df["own_number"] == b) & (rx_df["rx_number"] == a)].to_dict(
            orient='records'))

    init_rx_passive = first_or_none(rx_df[(rx_df["rx_round"] == r) & (rx_df["rx_slot"] == init_slot) & (
                rx_df["own_number"] == tdoa_src_dev_number) & (rx_df["rx_number"] == a)].to_dict(orient='records'))

    response_tx = first_or_none(
        tx_df[(tx_df["tx_round"] == r) & (tx_df["tx_slot"] == response_slot) & (tx_df["own_number"] == b)].to_dict(
            orient='records'))
    response_rx = first_or_none(
        rx_df[(rx_df["rx_round"] == r) & (rx_df["rx_slot"] == response_slot) & (rx_df["own_number"] == a) & (rx_df["rx_number"] == b)].to_dict(
            orient='records'))
    response_rx_passive = first_or_none(rx_df[(rx_df["rx_round"] == r) & (rx_df["rx_slot"] == response_slot) & (
                rx_df["own_number"] == tdoa_src_dev_number) & (rx_df["rx_number"] == b)].to_dict(orient='records'))

    final_tx = first_or_none(
        tx_df[(tx_df["tx_round"] == r) & (tx_df["tx_slot"] == final_slot) & (tx_df["own_number"] == a)].to_dict(
            orient='records'))
    final_rx = first_or_none(
        rx_df[(rx_df["rx_round"] == r) & (rx_df["rx_slot"] == final_slot) & (rx_df["own_number"] == b) & (rx_df["rx_number"] == a)].to_dict(
            orient='records'))
    final_rx_passive = first_or_none(rx_df[(rx_df["rx_round"] == r) & (rx_df["rx_slot"] == final_slot) & (
                rx_df["own_number"] == tdoa_src_dev_number) & (rx_df["rx_number"] == a)].to_dict(orient='records'))


    assert init_rx is None or init_rx['rx_number'] == a
    assert init_rx_passive is None or init_rx_passive['rx_number'] == a

    assert response_rx is None or response_rx['rx_number'] == b
    assert response_rx_passive is None or response_rx_passive['rx_number'] == b

    assert final_rx is None or final_rx['rx_number'] == a
    assert final_rx_passive is None or final_rx_passive['rx_number'] == a


    ret = {
        'init_tx': init_tx,
        'init_rx': init_rx,
        'init_rx_passive': init_rx_passive,
        'response_tx': response_tx,
        'response_rx': response_rx,
        'response_rx_passive': response_rx_passive,
        'final_tx': final_tx,
        'final_rx': final_rx,
        'final_rx_passive': final_rx_passive
    }

    return ret

def extract_record(testbed, rx_df, tx_df, r, a, b, tdoa_src_dev_number, init_slot, response_slot, final_slot, bias_corrected=True):
    data = extract_data_slots(rx_df, tx_df, r, a, b, tdoa_src_dev_number, init_slot, response_slot, final_slot)

    da = testbed.devs[a]
    db = testbed.devs[b]

    if tdoa_src_dev_number is not None:
        tdoa_src_dev = testbed.devs[tdoa_src_dev_number]
    else:
        tdoa_src_dev = None


    record = {}

    record['round'] = int(r)
    record['init_slot'] = init_slot
    record['response_slot'] = response_slot
    record['final_slot'] = final_slot
    record['initiator'] = a
    record['responder'] = b
    record['pair'] = "{}-{}".format(a, b)
    record['dist'] = get_dist(testbed.dev_positions[da], testbed.dev_positions[db]) if not hasattr(testbed, 'get_dist') else testbed.get_dist(da, db)
    record['tdoa'] = None

    # we check if data contains any None values, if so, we drop this exchange
    def dur(start, end):
        if None in [start, end]:
            return None

        if end <= start:
            end += 0xFFFFFFFFFF + 1  # we handle overflow here

        return end - start

    round_a = dur((data.get('init_tx', {}) or {}).get('tx_ts', None),
                  (data.get('response_rx', {}) or {}).get(
                      'bias_corrected_rx_ts' if bias_corrected else 'rx_ts', None))
    delay_b = dur(
        (data.get('init_rx', {}) or {}).get('bias_corrected_rx_ts' if bias_corrected else 'rx_ts', None),
        (data.get('response_tx', {}) or {}).get('tx_ts', None))
    delay_a = dur(
        (data.get('response_rx', {}) or {}).get('bias_corrected_rx_ts' if bias_corrected else 'rx_ts',
                                                None), (data.get('final_tx', {}) or {}).get('tx_ts', None))
    round_b = dur((data.get('response_tx', {}) or {}).get('tx_ts', None),
                  (data.get('final_rx', {}) or {}).get(
                      'bias_corrected_rx_ts' if bias_corrected else 'rx_ts', None))

    record['round_a'] = round_a
    record['delay_b'] = delay_b
    record['delay_a'] = delay_a
    record['round_b'] = round_b

    #record['init_rx_phase'] = (data.get('init_rx', {}) or {}).get('rx_ttcko_rc_phase', None)
    #record['init_rx_passive_phase'] = (data.get('init_rx_passive', {}) or {}).get('rx_ttcko_rc_phase', None)
    #record['response_rx_phase'] = (data.get('response_rx', {}) or {}).get('rx_ttcko_rc_phase', None)
    #record['response_rx_passive_phase'] = (data.get('response_passive_rx', {}) or {}).get('rx_ttcko_rc_phase', None)
    #record['final_rx_phase'] = (data.get('final_rx', {}) or {}).get('rx_ttcko_rc_phase', None)
    #record['final_rx_passive_phase'] = (data.get('final_rx_passive', {}) or {}).get('rx_ttcko_rc_phase', None)

    def ci_or_none_to_rd(ci):
        if ci is None:
            return None
        else:
            return ci_to_rd(ci)

    record['init_rd_cfo'] = ci_or_none_to_rd((data.get('init_rx', {}) or {}).get('ci', None))
    record['response_rd_cfo'] = ci_or_none_to_rd((data.get('response_rx', {}) or {}).get('ci', None))
    record['final_rd_cfo'] = ci_or_none_to_rd((data.get('final_rx', {}) or {}).get('ci', None))

    if None not in [round_a, delay_b, delay_a, round_b]:
        relative_drift = float(round_a + delay_a) / float(round_b + delay_b)
        twr_tof = convert_ts_to_m((round_a - relative_drift * delay_b) * 0.5)

        record['relative_drift'] = relative_drift
        record['twr_tof_ds'] = twr_tof

        record['twr_tof_ss'] = convert_ts_to_m((round_a - record['response_rd_cfo'] * delay_b) * 0.5) if record['response_rd_cfo'] is not None else None
        record['twr_tof_ss_reverse'] = convert_ts_to_m((round_b - record['final_rd_cfo'] * delay_a) * 0.5) if record['final_rd_cfo'] is not None else None

        if tdoa_src_dev:
            if not hasattr(testbed, 'get_dist'):
                record['tdoa'] = get_dist(testbed.dev_positions[da],
                                          testbed.dev_positions[tdoa_src_dev]) - get_dist(
                    testbed.dev_positions[tdoa_src_dev], testbed.dev_positions[db])
            else:
                record['tdoa'] = testbed.get_dist(da, tdoa_src_dev) - testbed.get_dist(tdoa_src_dev,db)
            record['tdoa_device'] = tdoa_src_dev_number

            passive_m_a = dur((data.get('init_rx_passive', {}) or {}).get(
                'bias_corrected_rx_ts' if bias_corrected else 'rx_ts', None),
                (data.get('response_rx_passive', {}) or {}).get(
                    'bias_corrected_rx_ts' if bias_corrected else 'rx_ts', None))
            passive_m_b = dur((data.get('response_rx_passive', {}) or {}).get(
                'bias_corrected_rx_ts' if bias_corrected else 'rx_ts', None),
                (data.get('final_rx_passive', {}) or {}).get(
                    'bias_corrected_rx_ts' if bias_corrected else 'rx_ts', None))

            if None not in [passive_m_a, passive_m_b]:
                record['passive_m_a'] = passive_m_a
                record['passive_m_b'] = passive_m_b

                record['tdoa_a_relative_drift_ds'] = (record['passive_m_a'] + record['passive_m_b']) / (
                        record['round_a'] + record['delay_a'])
                record['tdoa_b_relative_drift_ds'] = (record['passive_m_a'] + record['passive_m_b']) / (
                        record['round_b'] + record['delay_b'])

                record['tdoa_init_rd_cfo'] = ci_to_rd((data.get('init_rx_passive', {}) or {}).get('ci'))
                record['tdoa_response_rd_cfo'] = ci_to_rd(
                    (data.get('response_rx_passive', {}) or {}).get('ci'))
                record['tdoa_final_rd_cfo'] = ci_to_rd((data.get('final_rx_passive', {}) or {}).get('ci'))

                record['tdoa_est_ds'] = convert_ts_to_m(
                    0.5 * record['tdoa_a_relative_drift_ds'] * round_a + 0.5 * record[
                        'tdoa_b_relative_drift_ds'] * delay_b - passive_m_a)
                if None not in [record['tdoa_final_rd_cfo'], record['tdoa_response_rd_cfo']]:
                    record['tdoa_est_ss_init'] = convert_ts_to_m(
                        0.5 * record['tdoa_init_rd_cfo'] * round_a + 0.5 * record[
                            'tdoa_response_rd_cfo'] * delay_b - passive_m_a)
                    record['tdoa_est_ss_final'] = convert_ts_to_m(
                        0.5 * record['tdoa_final_rd_cfo'] * round_a + 0.5 * record[
                            'tdoa_response_rd_cfo'] * delay_b - passive_m_a)
                    record['tdoa_est_ss_both'] = convert_ts_to_m(
                        0.25 * record['tdoa_init_rd_cfo'] * round_a + 0.25 * record[
                            'tdoa_final_rd_cfo'] * round_a + 0.5 * record[
                            'tdoa_response_rd_cfo'] * delay_b - passive_m_a)
                    record['tdoa_est_mixed'] = convert_ts_to_m(
                        0.5 * record['tdoa_a_relative_drift_ds'] * round_a + 0.5 * record[
                            'tdoa_response_rd_cfo'] * delay_b - passive_m_a)
    return record




def gen_tdma_twr_records(testbed, run, tdoa_src_dev_number=None, bias_corrected=True, slots_per_phase=None):
    if slots_per_phase is None:
        slots_per_phase = testbed.SLOTS_PER_PHASE

    for (r, rx_events, tx_events) in gen_round_events(testbed, run):
        rx_df = pd.DataFrame.from_records(rx_events)
        tx_df = pd.DataFrame.from_records(tx_events)

        print(r, len(rx_events), len(tx_events))

        if len(rx_events) == 0 or len(tx_events) == 0:
            print("No Events for round")
            continue


        first_phase_txs = tx_df[tx_df['tx_slot'] < slots_per_phase]

        # we go through the relevant tx_events of the first phase
        for a_ev in first_phase_txs.to_dict(orient="records"):
            a = a_ev['own_number']
            a_dev = testbed.devs[a]

            init_slot = a_ev['tx_slot']
            final_slot = a_ev['tx_slot'] + slots_per_phase

            for b_ev in first_phase_txs.to_dict(orient="records"):
                b = b_ev['own_number']
                b_dev = testbed.devs[b]

                if a == b:
                    continue

                if a_ev['tx_slot'] < b_ev['tx_slot']:
                    # if a transmitted before b, we use the response in the same phase
                    response_slot = b_ev['tx_slot']
                else:
                    response_slot = b_ev['tx_slot'] + slots_per_phase # else we use the response in the next phase

                rec = extract_record(testbed, rx_df, tx_df, r, a, b, tdoa_src_dev_number, init_slot, response_slot, final_slot, bias_corrected=bias_corrected)
                # print(rec)
                yield rec


if __name__ == '__main__':
    import sys
    import json

    from testbed import lille
    testbed_name = sys.argv[1]
    run = sys.argv[2]

    if testbed_name == 'ds_kiel':
        from testbed import ds_kiel
        testbed = ds_kiel
    elif testbed_name == 'toulouse':
        from testbed import toulouse
        testbed = toulouse
    else:
        exit()

    recs = gen_tdma_twr_records(testbed, run, tdoa_src_dev_number=1)

    df = pd.DataFrame.from_records(recs)

    print(df)

    df.to_csv('out.csv')
