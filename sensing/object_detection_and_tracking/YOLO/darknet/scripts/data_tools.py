import os, glob
import xml.etree.ElementTree as ET
import shutil
import cv2
import numpy as np

classes = ["human", "car", "fire hydrant", "street sign"]
img_path = "data/"
xml_path = "xml/"
net_size = (416, 416)
img_format = 'jpg'


def resize_img(img_path):
    img_file_list = glob.glob(os.path.join(img_path, '*.%s' % img_format))
    img_resize_path = os.path.join(img_path, '..','train')
    if os.path.exists(img_resize_path):
        shutil.rmtree(img_resize_path)
    os.makedirs(img_resize_path)
    for i in img_file_list:
        img = cv2.imread(i, cv2.IMREAD_COLOR)
        img = cv2.resize(img, net_size)
        save_path = os.path.join(img_resize_path, os.path.basename(i))
        cv2.imwrite(save_path, img)


# asolute: left, top, right, bottom
# relative: cx, cy, w, h
def get_txt_from_xml():
    def pointToCenter(box):
        cx, cy = (box[:2] + box[2:]) / 2
        w, h = box[2:] - box[:2]
        return np.array((cx, cy, w, h))

    def encode(box):
        cx, w = box[::2] / net_size[0]
        cy, h = box[1::2] / net_size[1]
        return np.array((cx, cy, w, h))

    txt_path = os.path.abspath(os.path.join(xml_path, '..', 'txt_label'))
    if os.path.exists(txt_path):
        shutil.rmtree(txt_path)
    os.makedirs(txt_path)
    xml_file_list = glob.glob(os.path.join(xml_path, '*.xml'))
    for xml_file in xml_file_list:
        save_label_file = os.path.join(txt_path, os.path.basename(xml_file).replace('.xml', '.txt'))
        tree = ET.parse(xml_file)
        root = tree.getroot()
        size = root.find('size')
        w = int(size.find('width').text)
        h = int(size.find('height').text)
        for index, obj in enumerate(root.iter('object')):
            difficult = int(obj.find('difficult').text)
            cls = obj.find('name').text
            if cls not in classes or difficult == 1:
                continue
            cls_id = classes.index(cls)
            xmlbox = obj.find('bndbox')
            b = np.array((float(xmlbox.find('xmin').text), float(xmlbox.find('ymin').text),
                          float(xmlbox.find('xmax').text), float(xmlbox.find('ymax').text)))
            # get relative bb coordinate
            b[::2] = b[::2] * net_size[0] / w
            b[1::2] = b[1::2] * net_size[1] / h
            bb = encode(pointToCenter(b))

            with open(save_label_file, 'a') as f:
                f.write(str(cls_id) + " " + " ".join([str(a) for a in bb]))
                if index < len(root.findall('object'))-1:
                    f.write('\n')


def pair_img_txt():
    img_list = glob.glob(os.path.join(img_path, '..','train', '*.%s' % img_format))
    txt_path = os.path.join(img_path,'..', 'txt_label')
    for img_file in img_list:
        txt_file = os.path.join(txt_path, os.path.basename(img_file).replace(img_format, 'txt'))
        if os.path.exists(img_file.replace(img_format, 'txt')):
            os.remove(img_file.replace(img_format, 'txt'))
        if not os.path.exists(txt_file):
            with open(txt_file, 'w') as f:
                pass
        shutil.copyfile(txt_file, img_file.replace(img_format, 'txt'))


def main():
    resize_img(img_path)
    get_txt_from_xml()
    pair_img_txt()


if __name__ == '__main__':
    main()
