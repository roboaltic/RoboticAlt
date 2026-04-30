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
    def __init__(self, min_area=3000):
        self.min_area = min_area

    def detect(self, frame):
        result_frame = frame.copy()
        
        # 1. Ч/Б та сильне розмиття (щоб ігнорувати дрібні деталі всередині об'єкта)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (11, 11), 0)
        
        # 2. Пошук країв Canny
        edges = cv2.Canny(blur, 30, 100)
        
        # 3. Закриваємо розриви (робимо контур суцільним)
        # Ядро 9x9 потужно "зшиває" лінії об'єкта
        kernel = np.ones((9, 9), np.uint8)
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # 4. RETR_EXTERNAL - НАЙГОЛОВНІШЕ! Бере ТІЛЬКИ зовнішній, крайній силуэт
        contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area:
                # Згладжуємо контур
                epsilon = 0.01 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                
                # Малюємо синім кольором
                cv2.drawContours(result_frame, [approx], -1, (255, 0, 0), 3)
                
                x, y, w, h = cv2.boundingRect(approx)
                cv2.putText(result_frame, "OUTER EDGE", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                
        return result_frame