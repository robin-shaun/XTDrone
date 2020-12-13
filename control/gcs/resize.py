from PIL import Image

im = Image.open('robocup.jpg')
imBackground = im.resize((700, 470))  #700 470 840 564
imBackground.save('outdoor1_resize.jpg')