import os
import openai
import webbrowser
import urllib.request
from os import listdir

# openai.api_key = os.getenv("OPENAI_API_KEY")
openai.api_key = "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"
dir = "raw_imgs"
# for img_raw in os.listdir(dir):
img_raw = "11_1_3_square.png"
response = openai.Image.create_edit(
  image=open("raw_imgs/" + img_raw, "rb"),
  mask=open("11_1_5_mask_square.png", "rb"),
  prompt="color ground red",
  n=10,
  size="1024x1024"
)

img_url =response['data'][0]['url']
# urllib.request.urlretrieve(img_url, "gen_imgs/mask" + img_raw[0:6] + "_" + ".png")
webbrowser.open(img_url)
# print(img_url)