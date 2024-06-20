import matplotlib.pyplot as plt
from utility import slugify, cached_legacy, init_cache, load_env_config, set_global_cache_prefix_by_config

CONFIDENCE_FILL_COLOR = '0.8'
PERCENTILES_FILL_COLOR = '0.5'
COLOR_MAP = 'tab10'

PROTOCOL_NAME = "X"


def load_plot_defaults():
    # Configure as needed
    plt.rc('lines', linewidth=2.0)
    #plt.rc('image', cmap='viridis')
    plt.rc('legend', framealpha=1.0, fancybox=True)
    plt.rc('errorbar', capsize=3)
    plt.rc('pdf', fonttype=42)
    plt.rc('ps', fonttype=42)
    plt.rc('font', size=11)
    #plt.rc('font', size=8, family="serif", serif=['Times New Roman'] + plt.rcParams['font.serif'])
    plt.rcParams['axes.axisbelow'] = True


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


if __name__ == '__main__':

    config = load_env_config()

    load_plot_defaults()

    assert 'EXPORT_DIR' in config and config['EXPORT_DIR']

    if 'CACHE_DIR' in config and config['CACHE_DIR']:
        init_cache(config['CACHE_DIR'])
