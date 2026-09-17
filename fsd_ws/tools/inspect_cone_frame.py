#!/usr/bin/env python3
"""Print HSV connected components and ground-plane estimates for an RGB capture."""
import argparse
import cv2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image')
    args = parser.parse_args()
    frame = cv2.imread(args.image)
    if frame is None:
        raise ValueError('Image could not be loaded')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    import numpy as np
    for roi, data in [('blue cone', hsv[213:245, 55:75]), ('ground', hsv[230:290, 120:200]),
                      ('sky', hsv[20:150, :]), ('yellow cone', hsv[179:190, 280:286])]:
        print(roi, np.percentile(data.reshape(-1, 3), [10,50,90], axis=0).round(1))
    for name, lo, hi in [('blue', (100, 50, 60), (130, 255, 255)),
                         ('yellow', (20, 40, 70), (38, 255, 255))]:
        mask = cv2.inRange(hsv, lo, hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                               cv2.getStructuringElement(cv2.MORPH_RECT, (3, 7)))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) < 5:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if y + h < 160:
                continue
            depth = 302.8 * 1.045 / max(1, y+h-160)
            print(name, (x,y,w,h), 'aspect', round(h/w,2), 'depth_ground', round(depth,2),
                  'lateral', round(-(x+w/2-212)*depth/302.8,2),
                  'depth_height', round(302.8*.325/h,2))
    for s in [50, 70, 90, 105]:
        for v in [100, 120, 140]:
            mask = cv2.inRange(hsv, (100,s,v), (130,255,255))
            mask[:160] = 0
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                   cv2.getStructuringElement(cv2.MORPH_RECT, (3,7)))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            boxes = [cv2.boundingRect(c) for c in contours if cv2.contourArea(c) >= 5]
            print('blue S/V', s,v,boxes)


if __name__ == '__main__':
    main()
