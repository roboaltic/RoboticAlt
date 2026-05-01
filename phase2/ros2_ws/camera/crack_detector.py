import cv2
import numpy as np


class ObjectEdgeDetector:
    def __init__(self, min_area=2000, max_area=80000):
        self.min_area = min_area
        self.max_area = max_area

    def detect(self, frame):
        """
        Ищет границы крупных объектов.
        Возвращает: (обработанный кадр, список контуров объектов)
        """
        result_frame = frame.copy()
        found_objects = []

        # Предобработка для выделения границ
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (11, 11), 0)
        edges = cv2.Canny(blur, 30, 100)

        # Закрываем дыры в контурах
        kernel = np.ones((9, 9), np.uint8)
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.min_area < area < self.max_area:
                # Аппроксимация для сглаживания углов
                epsilon = 0.01 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)

                found_objects.append(approx)

                # Визуализация
                cv2.drawContours(result_frame, [approx], -1, (255, 0, 0), 3)
                x, y, w, h = cv2.boundingRect(approx)
                cv2.putText(
                    result_frame,
                    f"OBJ: {int(area)}px",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 0),
                    2,
                )

        return result_frame, found_objects


# class CrackDetector:
#     def __init__(self, min_crack_area=500):
#         self.min_crack_area = min_crack_area

#     def detect(self, roi_frame, parent_contours=None, offset=(0, 0)):
#         """
#         Ищет трещины только внутри переданных контуров.
#         roi_frame: кусок изображения (ROI)
#         parent_contours: контуры объектов с основного кадра
#         offset: (x, y) координаты левого верхнего угла ROI на основном кадре
#         """
#         result_frame = roi_frame.copy()
#         crack_count = 0

#         # 1. Алгоритм выделения тонких темных линий (Black-Hat)
#         gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
#         kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
#         blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)

#         # 2. Бинаризация и фильтрация шума
#         _, thresh = cv2.threshold(blackhat, 40, 255, cv2.THRESH_BINARY)
#         close_kernel = np.ones((5, 5), np.uint8)
#         closed_mask = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, close_kernel)

#         # Опционально: можно раскомментировать для отладки маски
#         # cv2.imshow("Debug Crack Mask", closed_mask)

#         contours, _ = cv2.findContours(
#             closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
#         )

#         for cnt in contours:
#             area = cv2.contourArea(cnt)
#             if area > self.min_crack_area:
#                 # Считаем центр масс трещины
#                 M = cv2.moments(cnt)
#                 if M["m00"] == 0:
#                     continue

#                 # Координаты центра трещины относительно ГЛОБАЛЬНОГО кадра
#                 cx = int(M["m10"] / M["m00"]) + offset[0]
#                 cy = int(M["m01"] / M["m00"]) + offset[1]

#                 # Проверка: попадает ли центр трещины в любой из контуров объектов
#                 is_inside = False
#                 if parent_contours:
#                     for obj_cnt in parent_contours:
#                         # >= 0 означает "внутри" или "на границе"
#                         if (
#                             cv2.pointPolygonTest(obj_cnt, (float(cx), float(cy)), False)
#                             >= 0
#                         ):
#                             is_inside = True
#                             break
#                 else:
#                     # Если объекты не заданы, считаем что можно искать везде
#                     is_inside = True

#                 if is_inside:
#                     crack_count += 1
#                     # Рисуем на локальном ROI
#                     cv2.drawContours(result_frame, [cnt], -1, (0, 0, 255), 2)
#                     x, y, w, h = cv2.boundingRect(cnt)
#                     cv2.rectangle(
#                         result_frame, (x, y), (x + w, y + h), (0, 255, 255), 1
#                     )
#                     cv2.putText(
#                         result_frame,
#                         f"CRACK: {int(area)}px",
#                         (x, y - 5),
#                         cv2.FONT_HERSHEY_SIMPLEX,
#                         0.5,
#                         (0, 0, 255),
#                         2,
#                     )

#         # Общий счетчик трещин на этом ROI
#         cv2.putText(
#             result_frame,
#             f"DETECTED: {crack_count}",
#             (10, 25),
#             cv2.FONT_HERSHEY_SIMPLEX,
#             0.7,
#             (0, 255, 0),
#             2,
#         )

#         return result_frame, crack_count > 0, crack_count


class CrackDetector:
    def __init__(self, min_crack_area=500, max_crack_area=5000):
        self.min_crack_area = min_crack_area
        self.max_crack_area = (
            max_crack_area  # Ограничим сверху, чтобы не обводить весь объект
        )

    def detect(self, roi_frame, parent_contours=None, offset=(0, 0)):
        result_frame = roi_frame.copy()
        crack_count = 0

        # 1. Улучшенный Black-Hat (ищем только очень тонкие линии)
        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
        # Уменьшаем ядро до 7x7 или 9x9, чтобы игнорировать толстые границы объекта
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)

        # 2. Жесткий порог + очистка
        _, thresh = cv2.threshold(blackhat, 50, 255, cv2.THRESH_BINARY)

        # Убираем мелкий мусор (соль и перец)
        thresh = cv2.medianBlur(thresh, 3)

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # Фильтр по площади: трещина не может быть размером с пол-объекта
            if self.min_crack_area < area < self.max_crack_area:
                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"]) + offset[0]
                cy = int(M["m01"] / M["m00"]) + offset[1]

                is_inside = False
                if parent_contours:
                    for obj_cnt in parent_contours:
                        # ВАЖНО: используем меру "расстояния" до края.
                        # Если результат > 5, значит точка глубоко внутри объекта (не на краю)
                        dist = cv2.pointPolygonTest(
                            obj_cnt, (float(cx), float(cy)), True
                        )
                        if dist > 5:  # Игнорируем зону 5 пикселей от края объекта
                            is_inside = True
                            break
                else:
                    is_inside = True

                if is_inside:
                    # Дополнительная проверка на вытянутость (трещина — это линия, а не круг)
                    x, y, w, h = cv2.boundingRect(cnt)
                    aspect_ratio = max(w, h) / min(w, h) if min(w, h) > 0 else 1

                    # Если объект слишком "квадратный", это может быть просто пятно
                    if aspect_ratio > 1.5:
                        crack_count += 1
                        cv2.drawContours(result_frame, [cnt], -1, (0, 0, 255), 2)
                        cv2.putText(
                            result_frame,
                            f"L:{int(area)}",
                            (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.4,
                            (0, 0, 255),
                            1,
                        )

        return result_frame, crack_count > 0, crack_count
