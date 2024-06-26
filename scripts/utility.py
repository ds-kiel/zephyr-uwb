import functools
import re
import gzip
import json
import os
import numpy as np

# https://stackoverflow.com/a/46801075/6669161
def slugify(obj, *args):
    if isinstance(obj, np.ndarray):
        return slugify(obj.tolist())
    if isinstance(obj, dict):
        res = ""
        for k, v in obj.items():
            res += slugify(k) + slugify(v)
    if isinstance(obj, list) or isinstance(obj, tuple):
        obj = " ".join(slugify(x) for x in obj)

    res = str(obj).strip().replace(' ', '_')


    if len(args) > 0:
        res += slugify(args)
    return re.sub(r'(?u)[^-\w.]', '', res)

def load_env_config():
    import dotenv
    return {
        **dotenv.dotenv_values(".env"),  # load shared development variables
        **dotenv.dotenv_values(".env.local"),  # load sensitive variables
        **os.environ,  # override loaded values with environment variables
    }

# Source: https://github.com/mpld3/mpld3/issues/434#issuecomment-340255689
class NumpyEncoder(json.JSONEncoder):
    """ Special json encoder for numpy types """
    def default(self, obj):
        if isinstance(obj, (np.int_, np.intc, np.intp, np.int8,
                            np.int16, np.int32, np.int64, np.uint8,
                            np.uint16, np.uint32, np.uint64)):
            return int(obj)
        elif isinstance(obj, (np.float_, np.float16, np.float32,
                              np.float64)):
            return float(obj)
        elif isinstance(obj,(np.ndarray,)):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


export_cache_dir = None

global_cache_prefix = ""

def set_global_cache_prefix(pref):
    global global_cache_prefix
    global_cache_prefix = slugify(pref)

def set_global_cache_prefix_by_config(config):
    set_global_cache_prefix(hash(json.dumps(config, sort_keys=True)))

def cached(func, id=None):
    global global_cache_prefix
    if export_cache_dir:
        def wrapped(*args, **kwargs):
            s = slugify(
                func.__name__, global_cache_prefix, args, kwargs
            ) if id is None else slugify(id)

            cache_file = os.path.join(export_cache_dir, s + '.json.gz')
            if not os.path.isfile(cache_file):
                data = func(*args, **kwargs)
                with gzip.open(cache_file, 'wt', encoding='UTF-8') as zipfile:
                    json.dump(data, zipfile, cls=NumpyEncoder)
            with gzip.open(cache_file, 'rt', encoding='UTF-8') as json_file:
                return json.load(json_file)

        return wrapped
    else:
        return func

def cached_legacy(id, proc_cb=None):
    return cached(proc_cb, id)


def cached_dt(id, proc_cb=None):
    import pandas as pd
    global global_cache_prefix
    if export_cache_dir:
        cache_file = os.path.join(export_cache_dir, slugify(id) + '.pkl.gz')
        if not os.path.isfile(cache_file):
            df = proc_cb()
            df.to_pickle(cache_file, compression='gzip')

        return pd.read_pickle(cache_file)
    else:
        return proc_cb()

def init_cache(path):
    global export_cache_dir
    export_cache_dir = path
    os.makedirs(export_cache_dir, exist_ok=True)


def in_parallel(callbacks, parallel=True):
    import threading

    res = None
    if parallel:
        res = [None for cb in callbacks]
        threads = []
        for i, cb in enumerate(callbacks):
            def wrapped_cb(idx, cb):
                res[idx] = cb()

            threads.append(threading.Thread(target=functools.partial(wrapped_cb, i, cb), args=()))

        for t in threads:
            t.start()

        for t in threads:
            t.join()
    else:
        res = [
            cb() for cb in callbacks
        ]
    return res


import subprocess
import os


DEFAULT_CROP = False

def save_and_crop(path, *args, **kwargs):

    filename, file_extension = os.path.splitext(path)

    crop = kwargs.pop('crop', DEFAULT_CROP)
    plt.savefig(path, *args, **kwargs)

    if crop:
        if file_extension == ".pdf":
            cropped_path = filename + "_cropped" + file_extension
            subprocess.run(["pdfcrop", path, cropped_path], stdout=subprocess.DEVNULL)