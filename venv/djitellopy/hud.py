import cv2
import numpy as np

# TrueType Fonts need special support in OpenCV
# We fall back to standard OpenCV if the support is not available
HAS_FREETYPE=hasattr(cv2,'freetype')
if hasattr(cv2,'freetype'):
    # font_name='Eurostile Bold Extended.ttf'
    font_name = 'OCRA.ttf'
    font = cv2.freetype.createFreeType2()
    font.loadFontData(fontFileName=font_name, id=0)
    font_size = 12
else:
    # using cv2.putText, the dimension of the font is given by font_scale
    # while font_size is used by the box function to compute the spacing between lines
    font= cv2.FONT_HERSHEY_DUPLEX
    font_scale=0.57
    font_size=int(23*font_scale)

font_anchor = {
    'top left': 'top left',
    'top right': 'top right',
    'bottom left': 'bottom left',
    'bottom right': 'bottom right'
}


def box(img, text, org, anchor=font_anchor['top left'], padding=5):
    """
    Output a (possibly multiline) aligned text at given coordinates
    """
    if isinstance(text, str):
        text = [text]

    nb_lines = len(text)
    for idx, line in enumerate(text):
        args={'text':line, 'thickness':-1}
        if HAS_FREETYPE:
            (line_width, _), _ = font.getTextSize(**args,
                                              fontHeight=font_size)
        else:
            (line_width, _), _ = cv2.getTextSize(**args,fontFace=font,fontScale=font_scale)

        if anchor == font_anchor['top left']:
            org_line = (org[0] + padding,
                        org[1] + (idx+1) * int(1.4 * font_size) + padding)
        elif anchor == font_anchor['top right']:
            org_line = (org[0] - line_width - padding, 
                        org[1] + (idx+1) * int(1.4 * font_size) + padding)
        elif anchor == font_anchor['bottom left']:
            org_line = (org[0] + padding,
                        org[1] + (idx - nb_lines + 1) * int(1.4 * font_size) -
                        font_size - padding)
        elif anchor == font_anchor['bottom right']:
            org_line = (org[0] - line_width - padding,
                        org[1] + (idx - nb_lines + 1) * int(1.4 * font_size) -
                        font_size - padding)
        args={'img':img,
                     'text':line,
                     'org':org_line,
                     'color':(0, 255, 0),
                     'thickness':1,
                     'bottomLeftOrigin':False}
        if HAS_FREETYPE:
            font.putText(**args,fontHeight=font_size,line_type=cv2.LINE_AA)
        else:
            cv2.putText(fontFace=font,fontScale=font_scale,**args)


def test():
    img_w, img_h = 960, 720
    img = np.zeros((img_h, img_w, 3), dtype=np.uint8)
    box(img, ['Top','left'], (0, 0))
    box(img, ['Top right',f'({img_w},0)'], (img_w, 0), font_anchor['top right'])
    box(img, f'(0,{img_h})', (0, img_h), font_anchor['bottom left'])
    box(img, ['Waiting...', 'Press any key'], (img_w, img_h),
         anchor=font_anchor['bottom right'])
    cv2.imshow('test', img)
    cv2.waitKey()

if __name__=='__main__':
    test()