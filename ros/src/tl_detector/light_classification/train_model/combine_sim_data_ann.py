#! /usr/bin/python
# create label_map for Bosch traffic light dataset
# Usage:
# python tl_label_map.py -annotation <PATH/train.yaml> -output <PATH/tl_label_map.pbtxt>

import sys
import argparse
    

def arg_parser(argv):
    """
    Input argument parser
    Args:
        argv: The input arguments
    """
    parser = argparse.ArgumentParser(description="Create label_map for Bosch Traffic Light Dataset")

    parser.add_argument("-data_dir", type=str, default='',
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
    f = open('ann_list.txt', 'w')

    ann_dirs = glob.glob(os.path.join(args.data_dir, 'ann_sim*'))
    for ann_dir in ann_dirs:
        xml_paths = glob.glob(os.path.join(args.data_dir, ann_dir, '*.xml'))
        for path in xml_paths:
            f.write(path + '/n')

    f.close()

if __name__ == '__main__':
    main(sys.argv)
