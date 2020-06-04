import yaml
from rosbag.bag import Bag

def get_duration(filename):

    info_dict = yaml.load(Bag(filename, 'r')._get_yaml_info())

    return info_dict['duration']
