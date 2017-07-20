import argparse
import rospkg

def params_setup(cmdline=None):
    parser = argparse.ArgumentParser()
    rospack = rospkg.RosPack()

    # path ctrl
    parser.add_argument('--models_path', type=str, default=rospack.get_path('roboy_intents_classification') + '/data/models/', help='path to the resources')
    parser.add_argument('--intents_path', default=rospack.get_path('roboy_intents_classification') + '/data/intents/', type=str, help='folder containing intents and example utterances')

    if cmdline:
        args = parser.parse_args(cmdline)
    else:
        args = parser.parse_args()

    return args
