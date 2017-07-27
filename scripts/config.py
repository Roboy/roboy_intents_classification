import argparse
import rospkg

def params_setup(cmdline=None):
    parser = argparse.ArgumentParser()
    rospack = rospkg.RosPack()

    # path ctrl
    parser.add_argument('--models_path', type=str, default=rospack.get_path('roboy_intents_classification') + '/data/models/', help='path to the resources')
    parser.add_argument('--intents_path', default=rospack.get_path('roboy_intents_classification') + '/data/intents/', type=str, help='folder containing intents and example utterances')
    parser.add_argument('__name')
    parser.add_argument('__log')
    parser.add_argument('--no-ros', dest='ros_enabled', action='store_false', help='switch to enable no ROS implementation')
    parser.set_defaults(ros_enabled=True)

    if cmdline:
        args = parser.parse_args(cmdline)
    else:
        args = parser.parse_args()

    return args
