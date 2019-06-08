import pyscreenshot as ImageGrab

if __name__ == '__main__':

    im = ImageGrab.grab(backend='pyqt')

    im.save('screenshot.png')

    im.show()