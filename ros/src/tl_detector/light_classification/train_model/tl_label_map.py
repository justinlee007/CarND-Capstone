#! /usr/bin/python
# create label_map for Bosch traffic light dataset
# Usage:
# python tl_label_map.py -annotation <PATH/train.yaml> -output <PATH/tl_label_map.pbtxt>

import sys
import argparse

import yaml
import tl_label_map_pb2

from google.protobuf import text_format
from tensorflow.python.lib.io import file_io


def create_pbtxt(unique_labels, output):
    tl_label_map = tl_label_map_pb2.TrafficLightMap()
    
    for i in range(len(unique_labels)):
        tl_label = tl_label_map.item.add()
        tl_label.id = i+1
        tl_label.name = unique_labels[i]
        
    file_io.atomic_write_string_to_file(output, text_format.MessageToString(tl_label_map))
        

def load_labels(yaml_path):
    """ get unique labels """
    
    unique_labels = []
    
    with open(yaml_path, 'r') as ymlfile:
        data = yaml.load(ymlfile)
        
    for item in data:
        boxes = item['boxes']
        for box in boxes:
            label = box['label']
            
            if label in unique_labels:
                continue
            unique_labels.append(label)
    
    print "total # of unique labels: ", len(unique_labels)
    print unique_labels
    return unique_labels
    

def arg_parser(argv):
    """
    Input argument parser
    Args:
        argv: The input arguments
    """
    parser = argparse.ArgumentParser(description="Create label_map for Bosch Traffic Light Dataset")

    parser.add_argument("-annotation", type=str, default='',
                    help="annotation file that includes traffic light labels")
    parser.add_argument("-output", type=str, default='',
                        help="output .pbtxt file that saves the label_map")
    args = parser.parse_args()
    print(args)
    if len(argv) == 1:
        parser.print_help()
        parser.exit()

    return args


def main(argv):
    # arg parse
    args = arg_parser(argv)

    unique_labels = load_labels(args.annotation)
    create_pbtxt(unique_labels, args.output)

if __name__ == '__main__':
    main(sys.argv)
