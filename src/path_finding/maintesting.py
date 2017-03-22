from PIL import Image
import numpy as np
import GenericSearchNode
import GenericAStar
import os
import webbrowser


# import matplotlib.image as img
# image = img.imread(file_name)
# print image.shape


def main():
    # binaryCamImg = np.load('binary_matrix.npy')
    # print binaryCamImg.shape
    # img = Image.new('1', binaryCamImg.shape)
    # pixels = img.load()
    #
    # for i in range(img.size[0]):
    #     for j in range(img.size[1]):
    #         pixelValue = binaryCamImg[i][j]
    #         pixels[i,j] = int(pixelValue)
    # img.save('robot_bw_view.png')
    # webbrowser.open('robot_bw_view.png')

    #data = np.fromfile('board-1-3.txt', None)
    #print data
    #txtfile = open('board-1-3.txt', 'r')
    #with open('board-1-3.txt') as file:

    Map = GenericAStar.Map('board-1-3.txt')
    Map.printMap()
    print Map.start
    AStar = GenericAStar.AStar(Map, "BestFS")
    AStar.best_first_search()
    print 'fuck yes'


    # Fix pixel.state: kan ikke vaare liste...



main()





    # os.system("pause")
    # for x in xrange(0,903):
    #     for y in xrange(0,971):
    #         print numpyArray[x][y]
    #img = Image.fromarray(numpyArray, '1')
    #img.save("robot_view.jpeg")
    #webbrowser.open('robot_view.jpeg')
# def main():
#     img =  Image.open('maze.jpg')
#     img_gray = img.convert('L')
#
#     # Let numpy do the heavy lifting for converting pixels to pure black or white
#     bw = np.asarray(img_gray).copy()
#
#     # Pixel range is 0...255, 256/2 = 128
#     bw[bw < 128] = 0  # Black
#     bw[bw >= 128] = 255  # White
#     # print image.bits, image.size, image.format
#
#     imfile = Image.fromarray(bw)
#     imfile.save("result_bw.png")
#     webbrowser.open('result_bw.png')
#     print bw[3]
#     # for x in xrange(img.size[0]):
#     #     for y in xrange(img.size[1]):
#     #         r, g, b =  img.getpixel((x,y))
#     #         print (r,g,b)
#     #         if g > 150 and r < 100 and b < 100:
#     #             os.system("pause")
#
#     #os.system("pause")
