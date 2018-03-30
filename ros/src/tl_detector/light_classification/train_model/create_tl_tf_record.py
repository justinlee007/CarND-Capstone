"""Convert the Bosch taffic light dataset to TFRecord for object_detection.

Example usage:
    python create_tl_tf_record.py --data_dir=<PATH> --output_dir=<PATH.record>
"""

import io
import logging
import os

import yaml
import numpy as np
import tensorflow as tf

from object_detection.utils import dataset_util
from object_detection.utils import label_map_util

flags = tf.app.flags
flags.DEFINE_string('data_dir', '', 'Root directory to raw pet dataset.')
flags.DEFINE_string('yaml_path', '', 'annotation path.')
flags.DEFINE_string('output_path', '', 'Path to directory to output TFRecords.')
flags.DEFINE_string('label_map_path', 'data/pet_label_map.pbtxt',
                    'Path to label map proto')
FLAGS = flags.FLAGS


def dict_to_tf_example(annotation, label_map_dict, data_dir):

    img_path = os.path.join(data_dir, annotation['path'])
    if not os.path.exists(img_path):
        img_path = os.path.join(data_dir, 'rgb', 'test', img_path.split('/')[-1])

    with tf.gfile.GFile(img_path, 'rb') as fid:
        encoded_png = fid.read()

    width = 1280
    height = 720

    xmins = []
    ymins = []
    xmaxs = []
    ymaxs = []
    classes = []
    classes_text = []

    for box in annotation['boxes']:
        xmins.append(float(box['x_min']) / width)
        ymins.append(float(box['y_min']) / height)
        xmaxs.append(float(box['x_max']) / width)
        ymaxs.append(float(box['y_max']) / height)
        classes.append(int(label_map_dict[box['label']]))
        classes_text.append(box['label'].encode())

    feature_dict = {'image/height': dataset_util.int64_feature(height), 
                    'image/width': dataset_util.int64_feature(width),
                    'image/filename': dataset_util.bytes_feature(img_path.encode('utf8')),
                    'image/source_id': dataset_util.bytes_feature(img_path.encode('utf8')),
                    'image/encoded': dataset_util.bytes_feature(encoded_png),
                    'image/format': dataset_util.bytes_feature('png'.encode('utf8')),
                    'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
                    'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
                    'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
                    'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
                    'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
                    'image/object/class/label': dataset_util.int64_list_feature(classes),}

    example = tf.train.Example(features=tf.train.Features(feature=feature_dict))
    return example


def create_tf_record(output_path, label_map_dict, yaml_path, data_dir):

    writer = tf.python_io.TFRecordWriter(output_path)

    with open(yaml_path, 'r') as ymlfile:
        annotation_yaml = yaml.load(ymlfile)

    for item in annotation_yaml:
        try:
            tf_example = dict_to_tf_example(item, label_map_dict, data_dir)
            writer.write(tf_example.SerializeToString())
        except ValueError:
            logging.warning('Invalid example: %s, ignoring.', item['path'])

    writer.close()


# TODO(derekjchow): Add test for pet/PASCAL main files.
def main(_):
    # raw data directory
    data_dir = FLAGS.data_dir
    # annotation path
    yaml_path = FLAGS.yaml_path
    output_path = FLAGS.output_path

    # load traffic light label_map
    label_map_dict = label_map_util.get_label_map_dict(FLAGS.label_map_path)

    logging.info('Reading from Bosch traffic light dataset.')

    create_tf_record(FLAGS.output_path, label_map_dict, yaml_path, data_dir)
  

if __name__ == '__main__':
    tf.app.run()



