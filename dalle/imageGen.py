import os
import openai
import webbrowser
import urllib.request
from os import listdir

# openai.api_key = os.getenv("OPENAI_API_KEY")
openai.api_key = "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"
response = openai.Image.create(
  # image=open("raw_imgs/" + img_raw, "rb"),
  # mask=open("11_1_5_mask_square.png", "rb"),
  prompt="building rooftop without garden hose",
  n=10,
  size="1024x1024"
)

img_url =response['data'][0]['url']
  # urllib.request.urlretrieve(img_url, "gen_imgs/openai_hose" + img_raw[0:6] + "_" + str(i) + ".png")
webbrowser.open(img_url)
# print(img_url)