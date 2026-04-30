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
    def __init__(self, min_area=8000):
        # Шукаємо тільки великі об'єкти (коробки, аркуші, перешкоди)
        self.min_area = min_area

    def detect(self, frame):
        result_frame = frame.copy()
        
        # 1. Ч/Б та розмиття
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (7, 7), 0)
        
        # 2. Пошук різких країв за алгоритмом Canny
        edges = cv2.Canny(blur, 50, 150)
        
        # 3. Потовщення ліній, щоб контур об'єкта не розривався
        kernel = np.ones((5, 5), np.uint8)
        dilated_edges = cv2.dilate(edges, kernel, iterations=1)
        
        # 4. Пошук зовнішніх контурів
        contours, _ = cv2.findContours(dilated_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area:
                # Апроксимація контуру (робить лінії більш прямими, ідеально для коробок/кутів)
                epsilon = 0.02 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                
                # Малюємо товстий синій контур навколо знайденого об'єкта
                cv2.drawContours(result_frame, [approx], -1, (255, 0, 0), 3)
                
                # Додаємо підпис
                x, y, w, h = cv2.boundingRect(approx)
                cv2.putText(result_frame, "OBJECT EDGE", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
        return result_frame