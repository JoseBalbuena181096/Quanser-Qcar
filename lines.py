from hal.utilities.image_processing import ImageProcessing
import numpy as np 
import cv2

class LaneDetect:
    def __init__(self) -> None:
       
        self.Source = np.float32([[270, 270], [550, 270], [0, 380], [820, 380]])
        self.Destination = np.float32([[270, 0], [550, 0], [270, 410], [550, 410]])
        self.polyright = [0,0,0]
        self.polyleft = [0,0,0]
        self.left_points = []
        self.right_points = []
        self.polyleft_last = [0,0,0]
        self.polyright_last = [0,0,0]
        self.error = 0
        self.matrix_perspective = []
        
		# Crop out a piece of the RGB to improve performance

    def TransformImage(self, frame):
        resize_frame = cv2.resize(frame,  (0, 0), fx=0.5, fy=0.5)
        original_img = resize_frame.copy()
        #print(resize_frame.shape)
        self.matrix_perspective = cv2.getPerspectiveTransform(self.Source, self.Destination)
        #for i in range(4):
        #    cv2.line(resize_frame, tuple(self.Source[i]), tuple(self.Source[(i + 1) % 4]), (0, 0, 255), 2)
        #    cv2.line(resize_frame, tuple(self.Destination[i]), tuple(self.Destination[(i + 1) % 4]), (0, 255, 0), 2)
        resize_frame  = cv2.warpPerspective( resize_frame , self.matrix_perspective, (resize_frame.shape[1], resize_frame.shape[0]))
		# Convert to HSV and then threshold it for yellow   
        hsvBuf = cv2.cvtColor(resize_frame, cv2.COLOR_BGR2HSV)
        binaryImage = ImageProcessing.binary_thresholding(frame= hsvBuf, lowerBounds=np.array([10, 50, 100]), upperBounds=np.array([45, 255, 255]))
        return original_img, resize_frame, binaryImage
    
    def histogram(self, binaryImage):
        histogram_lane = []
        lane_position = np.zeros(2, dtype=int)
        init_row, end_row = binaryImage.shape[0] // 2, binaryImage.shape[0] - 1

        for i in range(binaryImage.shape[1]):
            roi_lane = binaryImage[init_row :  end_row, i:i + 1]
            #roi_lane = binaryImage[ :, i:i + 1]
            histogram_lane.append(np.sum(roi_lane))

        left_ptr = np.argmax(histogram_lane[:binaryImage.shape[1] // 2])
        lane_position[0] = left_ptr

        right_ptr = np.argmax(histogram_lane[binaryImage.shape[1] // 2 + 1:]) + binaryImage.shape[1] // 2 + 1
        lane_position[1] = right_ptr

        return lane_position    
    
    def locate_lanes(self, img):
        region_interest = None
        self.left_points = []
        self.right_points = []
        nwindows = 12
        margin = 30
        minpix = 50
        win_y_low, win_y_high = 0, 0
        win_xleft_low, win_xleft_high = 0, 0
        win_xright_low, win_xright_high = 0, 0
        leftx_current, rightx_current = 0, 0
        mean_leftx, mean_rightx = 0, 0
        count_left, count_right = 0, 0
        locate_histogram = self.histogram(img)
        leftx_current, rightx_current = int(locate_histogram[0]), int(locate_histogram[1])

        window_height = img.shape[0] // nwindows

        for window in range(nwindows):
            mean_leftx, mean_rightx = 0, 0
            count_left, count_right = 0, 0

            win_y_low = img.shape[0] - (window + 1) * window_height
            win_y_high = img.shape[0] - window * window_height

            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            win_xleft_low = int(max(0, win_xleft_low + 1))
            win_xright_high = int(min(img.shape[1] - 1, win_xright_high))

            win_y_high = int(min(img.shape[0] - 1, win_y_high))
            win_y_low = int(max(1, win_y_low))

            for r in range(win_y_low, win_y_high):
                for cl in range(int(win_xleft_low) + 1, int(win_xleft_high)):
                    now_left_point = img[r, cl]
                    if now_left_point > 0:
                        add_left_point = (r, cl)
                        self.left_points.append(add_left_point)
                        mean_leftx += add_left_point[1]
                        count_left += 1

                for cr in range(int(win_xright_low) + 1, int(win_xright_high)):
                    now_right_point = img[r, cr]
                    if now_right_point > 0:
                        add_right_point = (r, cr)
                        self.right_points.append(add_right_point)
                        mean_rightx += add_right_point[1]
                        count_right += 1

            if count_left >= minpix:
                mean_leftx /= count_left
                leftx_current = mean_leftx

            if count_right >= minpix:
                mean_rightx /= count_right
                rightx_current = mean_rightx

    
    def regression_right(self):
        if len(self.right_points) < 75:
            return False

        x_values = np.array([point[0] for point in self.right_points])
        y_values = np.array([point[1] for point in self.right_points])

        coefficients = np.polyfit(x_values, y_values, 2)  # Ajusta un polinomio de segundo grado
        self.polyright = coefficients.tolist()

        return True

    
    def regression_left(self):
        if len(self.left_points) < 75:
            return False

        x_values = np.array([point[0] for point in self.left_points])
        y_values = np.array([point[1] for point in self.left_points])

        coefficients = np.polyfit(x_values, y_values, 2)  # Ajusta un polinomio de segundo grado
        self.polyleft = coefficients.tolist()

        return True
        
            

    def draw_lines(self, img, original_img):
        columnL, columnL_aux, columnR, columnR_aux, row, m, b, c, find_line_left, find_line_right, angle_to_mid_radian = 0,0,0,0,0,0,0,0,0,0,0
    
        find_line_right = self.regression_right()
        find_line_left = self.regression_left()
    
        self.right_points = []
        self.left_points = []
    
        center_cam = (img.shape[1] // 2) + 22
    
        if find_line_left and find_line_right:
            for row in range(img.shape[0] - 1, -1, -8):
                columnR = self.polyright[2] + self.polyright[1] * row + self.polyright[0] * (row * row)
                cv2.circle(img, (int(columnR), int(row)), int(4 / 2), (0, 255, 0), 2)
            
                columnL = self.polyleft[2] + self.polyleft[1] * row + self.polyleft[0] * (row * row)
                cv2.circle(img, (int(columnL), int(row)), int(4 / 2), (0, 255, 0), 2)
        
            center_lines = (columnR + columnL) / 2
        
                  
            distance_center = center_cam - center_lines

            cv2.line(img, (int(center_lines), 0), (int(center_cam), int(img.shape[0] - 1)), (0, 255, 0), 2)
        
            for k in range(3):
                self.polyleft_last[k] = self.polyleft[k]
                self.polyright_last[k] = self.polyright[k]
    
        elif find_line_left:
            for row in range(img.shape[0] - 1, -1, -8):
                columnL = self.polyleft[2] + self.polyleft[1] * row + self.polyleft[0] * (row * row)
                cv2.circle(img, (int(columnL), int(row)), int(4 / 2), (0, 255, 0), 2)
        
            
            columnL_aux = self.polyleft[2]
            center_lines = columnL_aux  - 125
            distance_center = center_cam - center_lines
        
            for k in range(3):
                self.polyleft_last[k] = self.polyleft[k]
            print("Lane Left")
    
        elif find_line_right:
            for row in range(img.shape[0] - 1, -1, -8):
                columnR = self.polyright[2] + self.polyright[1] * row + self.polyright[0] * (row * row)
                cv2.circle(img, (int(columnR), int(row)), int(4 / 2), (0, 255, 0), 2)
        
            
            columnR_aux = self.polyright[2]
        
            center_lines =    columnR_aux + 125
        
            distance_center = center_cam - center_lines
        
            for k in range(3):
                self.polyright[k] = self.polyright_last[k]
            print("Lane Right")
    
        if not find_line_left and not find_line_right:
            for row in range(img.shape[0] - 1, -1, -8):
                columnR = self.polyright_last[2] + self.polyright_last[1] * row + self.polyright_last[0] * (row * row)
                cv2.circle(img, (int(columnR), int(row)), int(4 / 2), (0, 255, 0), 2)
            
                columnL = self.polyleft_last[2] + self.polyleft_last[1] * row + self.polyleft_last[0] * (row * row)
                cv2.circle(img, (int(columnL), int(row)), int(4 / 2), (0, 255, 0), 2)
        
            center_lines = (columnR + columnL) / 2
        
                   
            distance_center = center_cam - center_lines
            
        if center_lines > img.shape[1]:
            center_lines = img.shape[1] - 1
        elif center_lines < 0:
            center_lines = 0
        cv2.line(img, (int(center_cam), int(img.shape[0] / 4)), (int(center_cam), int(img.shape[0] * 3 / 4)), (0, 255, 0), 2)
        cv2.line(img, (int(center_lines), 0), (int(center_cam), int(img.shape[0] - 1)), (0, 0, 255), 2)
        self.error = distance_center
        invertedPerspectiveMatrix = np.linalg.inv(self.matrix_perspective)
        warped_frame = cv2.warpPerspective(img, invertedPerspectiveMatrix, (img.shape[1], img.shape[0]))
        img = cv2.bitwise_xor(original_img, warped_frame)
        #img = cv2.resize(img,  (0, 0), fx=0.5, fy=0.5)
        return img
    
    def find_lines(self, frame):
        original_img, resize_frame, image_black = self.TransformImage(frame)
        self.locate_lanes(image_black)
        self.regression_left()
        self.regression_right() 
        return self.draw_lines(resize_frame, original_img)
