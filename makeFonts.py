import os

import numpy as np

from PIL import Image, ImageFont, ImageDraw


def process_char(font, char):
    # Calculate char info
    char_size = font.getsize(char)
    char_mask = font.getmask(char)
    # Create image
    char_image = Image.new('1', char_size, 0)
    # Draw font to image
    d = ImageDraw.Draw(char_image)
    d.fontmode = '1'
    d.text((0, 0), char, font=font, fill=(1))
    return char_image


def generate_bin_image(image):
    data = image.getdata()
    size = image.size

    bin_image = [[0 for i in range(size[0])] for j in range(size[1])]

    for j in range(size[1]):
        for i in range(size[0]):
            index = j * size[0] + i
            if data[index] == 0:
                bin_image[j][i] = '0'
            else:
                bin_image[j][i] = '1'

    return bin_image


def generatePixelList(binaryImage):
    # Convert into on-list only
    onList = []
    count = 0
    for row in binaryImage:
        for col in row:
            if col == "1":
                onList.append(count)
            count += 1
    return onList


def makeLookupList(lengths):
    template = """
uint16_t const fontLocation[] = {%s};
"""
    locations = np.cumsum(lengths)

    return template % (", ".join(["0"] + [str(x) for x in locations][:-1]).rstrip(", "))


def makeNumPixelsList(lengths):
    template = """
uint8_t const fontLength[] = {%s};
"""
    joiner = ", "
    return template % (joiner.join([str(x) for x in lengths]).rstrip(joiner))


def makeDataList(onValues):
    template = """
uint8_t const fontValues[] = {%s};
"""
    flatOnValues = [
        pixelNumber for sublist in onValues for pixelNumber in sublist]

    joiner = ", "
    return template % (joiner.join([str(x) for x in flatOnValues]).rstrip(joiner))


def makeHeightList(heights):
    template = """
uint8_t const fontHeight[] = {%s};
"""
    joiner = ", "
    return template % (joiner.join([str(x) for x in heights]).rstrip(joiner))


def makeWidthList(widths):
    template = """
uint8_t const fontWidth[] = {%s};
"""
    joiner = ", "
    return template % (joiner.join([str(x) for x in widths]).rstrip(joiner))


def functions(start):
    offsetFunction = """
int fontOffsetLookup(){
    return %d;
}
""" % (-start)

    lookupFunctions = """
uint8_t fontHeightGet(char ch){
    return fontHeight[((uint8_t) ch) + fontOffsetLookup()];
}
uint8_t fontWidthGet(char ch){
    return fontWidth[((uint8_t) ch) + fontOffsetLookup()];
}
uint8_t fontNumberPixelsGet(char ch){
    return fontLength[((uint8_t) ch) + fontOffsetLookup()];
}

uint8_t* fontGet(char ch){
    return (uint8_t*) &( fontValues[fontLocation[((uint8_t) ch) + fontOffsetLookup()]] );
}

"""

    return offsetFunction + lookupFunctions


if __name__ == "__main__":
    # Make an ascii table

    start = 65
    end = 122

    font = ImageFont.truetype("comic.ttf", size=10)

    onValues = []
    widths = []
    heights = []

    for charIndex in range(start, end+1):
        char = chr(charIndex)
        image = process_char(font, char)
        binImage = generate_bin_image(image)
        widths.append(len(binImage[0]))
        heights.append(len(binImage))
        onValues.append(generatePixelList(binImage))

    with open(os.path.join("src", "fonts.h"), "w") as f:
        f.write("""#include <stdint.h>""")
        f.write(makeLookupList([len(x) for x in onValues]))
        f.write(makeNumPixelsList([len(x) for x in onValues]))
        f.write(makeWidthList(widths))
        f.write(makeHeightList(heights))
        f.write(makeDataList(onValues))
        f.write(functions(start))
