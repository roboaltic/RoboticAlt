import cv2
import numpy as np

class CrackDetector:
    def __init__(self):
        pass # Ліміт в 1000 пікселів тепер прописаний жорстко нижче

    def detect(self, frame):
        result_frame = frame.copy()
        crack_count = 0

        # Градації сірого та Black-Hat для пошуку тонких темних ліній
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)

        # Відсікаємо блідий шум
        _, thresh = cv2.threshold(blackhat, 40, 255, cv2.THRESH_BINARY)
        close_kernel = np.ones((5, 5), np.uint8)
        closed_mask = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, close_kernel)

        cv2.imshow("Crack Mask (DEBUG)", closed_mask)

        # Пошук контурів тріщин
        contours, _ = cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # ЖОРСТКИЙ ЛІМІТ: ігноруємо все, що менше 1000 пікселів
            if area > 1000:
                crack_count += 1
                cv2.drawContours(result_frame, [cnt], -1, (0, 0, 255), 2)
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(result_frame, (x, y), (x + w, y + h), (0, 255, 255), 1)
                cv2.putText(result_frame, f"CRACK: {int(area)}px", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.putText(result_frame, f"CRACKS DETECTED: {crack_count}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        crack_found = crack_count > 0

        return result_frame, crack_found, crack_count

class ObjectEdgeDetector:
    def __init__(self, min_area=2000, max_area=80000): # Додали верхню межу (max_area)
        self.min_area = min_area
        self.max_area = max_area

    def detect(self, frame):
        result_frame = frame.copy()
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (11, 11), 0)
        edges = cv2.Canny(blur, 30, 100)
        
        kernel = np.ones((9, 9), np.uint8)
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # === ФІЛЬТРАЦІЯ ПО ПЛОЩІ (Від і До) ===
            if self.min_area < area < self.max_area:
                epsilon = 0.01 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                
                # Малюємо контур
                cv2.drawContours(result_frame, [approx], -1, (255, 0, 0), 3)
                
                # Виводимо точну площу на екран
                x, y, w, h = cv2.boundingRect(approx)
                cv2.putText(result_frame, f"OBJECT: {int(area)}px", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                
        return result_frame