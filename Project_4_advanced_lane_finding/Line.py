from collections import deque
import numpy as np
import cv2
class Line():
    (ym_per_pixel, xm_per_pixel) = (30/720, 3.7/700) # meters per pixel in y and x dimension
    def __init__(self):
        # last 5 iterations detection
        self.detected = False
        # last 10 line fits in pixel([A,B,C]...) for Ax**2 + Bx + C
        self.recent_fits = deque(maxlen=15)
        # last 10 line fits in meter([A,B,C]...) for Ax**2 + Bx + C
        self.recent_fits_meter = deque(maxlen=15)
        # store all x and y pixel coordinates for calculation
        self.x = None
        self.y = None
    def update(self, detected, x, y):
        self.detected = detected
        if detected:
            self.x = x
            self.y = y
            self.recent_fits_meter.append(self.poly_fit_meter)
            self.recent_fits.append(self.poly_fit)
        elif len(self.recent_fits) != 0:
            self.recent_fit_meter.append(self.ave_poly_fits_meter)
            self.recent_fit.append(self.ave_poly_fits) # use the last time fit for the current frame
    def draw(self, mask, line_width=50, color=[255,0,0]):
        """use the average fit coefficient to draw line on the mask"""
        # Generate x and y values for plotting
        plot_y = np.linspace(0, mask.shape[0]-1, mask.shape[0])
        fit_x = self.ave_poly_fit[0]*plot_y**2 + self.ave_poly_fit[1]*plot_y + self.ave_poly_fit[2]

        line_left_side = fit_x - line_width // 2
        line_right_side = fit_x + line_width // 2

        # recast the x and y points to from an rectangular array usable format for cv2.fillPoly()
        pts_left = np.array(list(zip(line_left_side, plot_y)))
        pts_right = np.array(np.flipud(list(zip(line_right_side, plot_y))))
        pts = np.vstack([pts_left, pts_right])
        # Draw the lane onto the warped blank image
        return cv2.fillPoly(mask, [np.int32(pts)], color)
    @property
    def poly_fit(self):
        return np.polyfit(self.y, self.x, 2)
    @property
    # averaged polynomial coefficient of the last n iteractions
    def ave_poly_fit(self):
        return np.mean(self.recent_fits, axis=0)
    @property
    def poly_fit_meter(self):
        return np.polyfit(self.y*self.ym_per_pixel, self.x*self.xm_per_pixel, 2)
    @property
    # averaged polynomial coefficient of the last n iteractions in meter
    def ave_poly_fit_meter(self):
        return np.mean(self.recent_fits_meter, axis=0)
    @property
    def curvature_meter(self):
        return ((1 + (2*self.ave_poly_fit_meter[0]*720*self.ym_per_pixel + self.ave_poly_fit_meter[1])**2)**1.5) / np.absolute(2*self.ave_poly_fit_meter[0])