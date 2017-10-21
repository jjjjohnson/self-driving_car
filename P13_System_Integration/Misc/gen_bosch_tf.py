"""
Usage:
  # From tensorflow/models/
  # Create train data:
  python gen_bosch_tf.py --yaml_input=data/train.yaml  --img_dir_path=images --output_path=train.record
"""

import tensorflow as tf
import yaml
import os
from object_detection.utils import dataset_util


flags = tf.app.flags
flags.DEFINE_string('yaml_input', '', 'Path to the yaml input')
flags.DEFINE_string('img_dir_path', '', 'Path to the image dir')
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
FLAGS = flags.FLAGS

LABEL_DICT =  {
    "Green" : 1,
    "Red" : 2,
    "GreenLeft" : 3,
    "GreenRight" : 4,
    "RedLeft" : 5,
    "RedRight" : 6,
    "Yellow" : 7,
    "off" : 8,
    "RedStraight" : 9,
    "GreenStraight" : 10,
    "GreenStraightLeft" : 11,
    "GreenStraightRight" : 12,
    "RedStraightLeft" : 13,
    "RedStraightRight" : 14
    }

def create_tf_example(example):
    
    # Bosch
    height = 720 # Image height
    width = 1280 # Image width

    filename = example['path'] # Filename of the image. Empty if image is not from file
    filename = filename.encode()

    with tf.gfile.GFile(example['path'], 'rb') as fid:
        encoded_image = fid.read()

    image_format = 'png'.encode() 

    xmins = [] # List of normalized left x coordinates in bounding box (1 per box)
    xmaxs = [] # List of normalized right x coordinates in bounding box
                # (1 per box)
    ymins = [] # List of normalized top y coordinates in bounding box (1 per box)
    ymaxs = [] # List of normalized bottom y coordinates in bounding box
                # (1 per box)
    classes_text = [] # List of string class name of bounding box (1 per box)
    classes = [] # List of integer class id of bounding box (1 per box)

    for box in example['boxes']:
        #if box['occluded'] is False:
        #print("adding box")
        xmins.append(float(box['x_min'] / width))
        xmaxs.append(float(box['x_max'] / width))
        ymins.append(float(box['y_min'] / height))
        ymaxs.append(float(box['y_max'] / height))
        classes_text.append(box['label'].encode())
        classes.append(int(LABEL_DICT[box['label']]))


    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_image),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(classes),
    }))

    return tf_example


def main(_):
    
    writer = tf.python_io.TFRecordWriter(FLAGS.output_path)
    
    # BOSCH
    #INPUT_YAML = "yaml/train.yaml"
    #examples = yaml.load(open(INPUT_YAML, 'rb').read())
    examples = yaml.load(open(FLAGS.yaml_input, 'rb').read())

    #examples = examples[:10]  # for testing
    len_examples = len(examples)
    print("Loaded ", len(examples), "examples")

    for i in range(len(examples)):
        #path = os.path.join(os.getcwd(), FLAGS.img_dir_path)
        #examples[i]['path'] = os.path.abspath(os.path.join(os.path.dirname(INPUT_YAML), examples[i]['path']))
        examples[i]['path'] = os.path.abspath(os.path.join(FLAGS.img_dir_path, examples[i]['path']))
    
    counter = 0
    for example in examples:
        tf_example = create_tf_example(example)
        writer.write(tf_example.SerializeToString())

        if counter % 10 == 0:
            print("Percent done", (float(counter)/len_examples)*100.0)
        counter += 1

    writer.close()



if __name__ == '__main__':
    tf.app.run()
