from PIL import Image

im = Image.open('indoor1.jpg')
imBackground = im.resize((875, 587))  #700 470 840 564
imBackground.save('indoor1_resize.jpg')
