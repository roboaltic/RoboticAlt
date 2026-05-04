import cv2
import numpy as np


class VisionAnalytic:
    """Базовые утилиты для обработки"""

    @staticmethod
    def get_mask(frame, lower, upper):
        return cv2.inRange(frame, lower, upper)


class ObjectEdgeDetector:
    def __init__(self):
        self.params = {"min_area": 5000, "kernel": np.ones((15, 15), np.uint8)}

    def get_object_mask(self, frame):
        # Быстрое выделение контуров объектов
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blk = cv2.GaussianBlur(gray, (7, 7), 0)
        edge = cv2.Canny(blk, 50, 150)
        # Замыкаем объект в сплошную маску
        mask = cv2.morphologyEx(edge, cv2.MORPH_CLOSE, self.params["kernel"])
        # Заполняем пустоты внутри
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obj_mask = np.zeros_like(mask)
        valid_cnts = [
            c for c in contours if cv2.contourArea(c) > self.params["min_area"]
        ]
        cv2.drawContours(obj_mask, valid_cnts, -1, 255, -1)
        return obj_mask, valid_cnts


class CrackDetector:
    def __init__(self):
        self.k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

    def detect(self, frame, obj_mask):
        # 1. Выделяем все темные линии (Blackhat)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, self.k)
        _, thresh = cv2.threshold(hat, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # 2. Главная фишка: Оставляем только то, что внутри маски объекта
        refined_mask = cv2.bitwise_and(thresh, obj_mask)

        # 3. Фильтрация шума и поиск контуров трещин
        contours, _ = cv2.findContours(
            refined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cracks = []
        for c in contours:
            if 100 < cv2.contourArea(c) < 5000:
                # Проверка на вытянутость (Линия, а не пятно)
                rect = cv2.minAreaRect(c)
                w, h = rect[1]
                if max(w, h) / (min(w, h) + 0.1) > 2.0:
                    cracks.append(c)
        return cracks
