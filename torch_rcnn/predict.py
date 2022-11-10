import os
import numpy as np
import torch
from PIL import Image

import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor

from engine import train_one_epoch, evaluate
import utils
import transforms as T
from train_model import get_transform
from pennfudan_dataset import PennFudanDataset
from hose_dataset import HoseDataset


def predict():
    device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

    # dataset = PennFudanDataset('PennFudanPed', get_transform(train=True))
    # dataset_test = PennFudanDataset('PennFudanPed', get_transform(train=False))

    dataset = HoseDataset("hose_dataset", get_transform(train=True))
    dataset_test = HoseDataset("hose_dataset", get_transform(train=False))

    # split the dataset in train and test set
    indices = torch.randperm(len(dataset)).tolist()
    dataset = torch.utils.data.Subset(dataset, indices[:-2])
    dataset_test = torch.utils.data.Subset(dataset_test, indices[-2:])

    model = torch.load("example_model.pt")
    print("model loaded")
    img, _ = dataset_test[0]
    model.eval()
    with torch.no_grad():
        prediction = model([img.to(device)])
        output_img = Image.fromarray(img.mul(255).permute(1, 2, 0).byte().numpy())
        mask = Image.fromarray(prediction[0]['masks'][0, 0].mul(255).byte().cpu().numpy())
        # print(output_img)
        # print(mask)
        output_img.show()
        mask.show()

if __name__ == "__main__":
        predict()