# encoding : utf-8
import cv2
import numpy as np
from picamera2 import Picamera2  # camera


class detector:
    def __init__(self):
        self.cone_ratio = 33 / 70
        self.ratio_thresh = 0.1
        self.detect_cone_flag = True  # if False, only parachute detection is used
        self._min_component_occupancy = 0.001
        self._cone_reached_occupancy = 0.15
        self._cone_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        self._parachute_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))

        # cone detection outputs (existing API)
        self.input_img = None
        self.projected_img = None
        self.binarized_img = None
        self.detected = None
        self.probability = None
        self.centroids = None
        self.cone_direction = None
        self.occupancy = None
        self.is_detected = None
        self.is_reached = None

        # parachute detection outputs (new)
        self.parachute_projected_img = None
        self.parachute_binarized_img = None
        self.parachute_detected = None
        self.parachute_centroids = None
        self.parachute_direction = None
        self.is_parachute_detected = None

        self.picam2 = None

    def set_roi_img(self):
        rois = ["./libs/roi1.png", "./libs/roi2.png", "./libs/roi3.png"]
        weights = [2.0, 1.0, 1.0]

        alpha_thresh = 200
        h_bins, s_bins = 180, 256
        mixed = np.zeros((h_bins, s_bins), np.float32)

        for path, w in zip(rois, weights):
            rgba = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if rgba is None:
                raise FileNotFoundError(path)
            if rgba.ndim != 3 or rgba.shape[2] != 4:
                raise ValueError(f"ROI must be RGBA (HxWx4). got: shape={rgba.shape}")

            bgr = cv2.cvtColor(rgba, cv2.COLOR_BGRA2BGR)
            alpha = rgba[:, :, 3]
            mask = (alpha >= alpha_thresh).astype(np.uint8) * 255
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

            hist = cv2.calcHist(
                images=[hsv],
                channels=[0, 1],
                mask=mask,
                histSize=[h_bins, s_bins],
                ranges=[0, 180, 0, 256],
            ).astype(np.float32)

            s = float(hist.sum())
            if s > 0:
                hist /= s
                mixed += float(w) * hist

        mixed_sum = float(mixed.sum())
        if mixed_sum <= 0:
            raise RuntimeError(
                "Mixed histogram is empty. Check alpha masks / threshold."
            )
        mixed /= mixed_sum

        self.__cone_roi_hist = mixed
        cv2.normalize(
            self.__cone_roi_hist, self.__cone_roi_hist, 0, 255, cv2.NORM_MINMAX
        )

        parachute_roi = cv2.imread("./libs/parachute_roi.png", cv2.IMREAD_UNCHANGED)
        if parachute_roi is None:
            raise FileNotFoundError("./libs/parachute_roi.png")
        if parachute_roi.ndim != 3 or parachute_roi.shape[2] != 4:
            raise ValueError(
                f"Parachute ROI must be RGBA (HxWx4). got: shape={parachute_roi.shape}"
            )

        bgr = cv2.cvtColor(parachute_roi, cv2.COLOR_BGRA2BGR)
        alpha = parachute_roi[:, :, 3]
        mask = (alpha >= alpha_thresh).astype(np.uint8) * 255
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        self.__parachute_roi_hist = cv2.calcHist(
            images=[hsv],
            channels=[0, 1],
            mask=mask,
            histSize=[h_bins, s_bins],
            ranges=[0, 180, 0, 256],
        ).astype(np.float32)
        cv2.normalize(
            self.__parachute_roi_hist,
            self.__parachute_roi_hist,
            0,
            255,
            cv2.NORM_MINMAX,
        )

    def set_cone_ratio(self, ratio):
        self.cone_ratio = ratio

    def __get_camera_img(self):
        if self.picam2 is None:
            self.picam2 = Picamera2()
            cam_conf = self.picam2.create_preview_configuration(
                main={"size": (640, 480), "format": "RGB888"},
                lores={"size": (320, 240), "format": "YUV420"},
            )
            self.picam2.configure(cam_conf)
            self.picam2.start()
            print("camera configured")

        if self.detect_cone_flag:
            self.input_img = cv2.blur(self.picam2.capture_array("main"), (8, 8))
        else:
            lores_img = cv2.cvtColor(
                self.picam2.capture_array("lores"), cv2.COLOR_YUV2BGR_NV12
            )
            self.input_img = cv2.blur(lores_img, (8, 8))

    def detect(self):
        self.__get_camera_img()

        if self.detect_cone_flag:
            self._clear_parachute_result()
            self.is_parachute_detected = False
            self.__back_projection()
            self.__binarization()
            self.__find_cone_centroid()
            return

        self.__detect_parachute()
        self._clear_cone_result()
        self.is_detected = False
        self.is_reached = False
        return

    def __back_projection(self):
        img_hsv = cv2.cvtColor(self.input_img, cv2.COLOR_BGR2HSV)
        self.projected_img = cv2.calcBackProject(
            [img_hsv], [0, 1], self.__cone_roi_hist, [0, 180, 0, 256], 1
        )

    def __parachute_back_projection(self):
        img_hsv = cv2.cvtColor(self.input_img, cv2.COLOR_BGR2HSV)
        self.parachute_projected_img = cv2.calcBackProject(
            [img_hsv], [0, 1], self.__parachute_roi_hist, [0, 180, 0, 256], 1
        )

    def __binarization(self):
        _, th = cv2.threshold(
            self.projected_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        self.binarized_img = cv2.morphologyEx(th, cv2.MORPH_DILATE, self._cone_kernel)

    def __parachute_binarization(self):
        _, th = cv2.threshold(
            self.parachute_projected_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        self.parachute_binarized_img = cv2.morphologyEx(
            th, cv2.MORPH_DILATE, self._parachute_kernel
        )

    def __detect_parachute(self):
        self.__parachute_back_projection()
        self.__parachute_binarization()
        self.__find_parachute_centroid()

    def __find_parachute_centroid(self):
        img_size = (
            self.parachute_binarized_img.shape[0]
            * self.parachute_binarized_img.shape[1]
        )
        nlabels, _, stats, centroids = cv2.connectedComponentsWithStats(
            self.parachute_binarized_img.astype(np.uint8)
        )
        if nlabels == 1:
            self._clear_parachute_result()
            self.is_parachute_detected = False
            return

        stats = stats[1:, :]
        centroids = centroids[1:, :]
        occupancies = stats[:, cv2.CC_STAT_AREA] / img_size

        max_occupancy = np.max(occupancies)
        idx_parachute = (
            np.argmax(occupancies)
            if max_occupancy > self._min_component_occupancy
            else -1
        )
        self.is_parachute_detected = idx_parachute >= 0
        if not self.is_parachute_detected:
            self._clear_parachute_result()
            return

        self.parachute_detected = stats[idx_parachute, :]
        self.parachute_centroids = centroids[idx_parachute]
        self.parachute_direction = (
            self.parachute_centroids[0] / self.parachute_binarized_img.shape[1]
        )

    def __find_cone_centroid(self):
        img_size = self.binarized_img.shape[0] * self.binarized_img.shape[1]
        nlabels, _, stats, centroids = cv2.connectedComponentsWithStats(
            self.binarized_img.astype(np.uint8)
        )
        if nlabels == 1:
            self._clear_cone_result()
            self.is_detected = False
            self.is_reached = False
            return

        stats = stats[1:, :]
        centroids = centroids[1:, :]

        probabilities = np.abs(
            stats[:, cv2.CC_STAT_WIDTH] / stats[:, cv2.CC_STAT_HEIGHT] - self.cone_ratio
        )
        occupancies = stats[:, cv2.CC_STAT_AREA] / img_size
        max_occupancy = np.max(occupancies)
        idx_cone = (
            np.argmax(occupancies)
            if max_occupancy > self._min_component_occupancy
            else -1
        )

        self.is_detected = idx_cone >= 0

        if max_occupancy > self._cone_reached_occupancy:
            self.is_reached = True
            self.picam2.capture_file("./log/capture_img.png")
        else:
            self.is_reached = False

        if not self.is_detected:
            self._clear_cone_result()
            return

        self.occupancy = occupancies[idx_cone]
        self.detected = stats[idx_cone, :]
        self.centroids = centroids[idx_cone]
        self.probability = probabilities[idx_cone]
        self.cone_direction = self.centroids[0] / self.binarized_img.shape[1]

    def _clear_cone_result(self):
        self.detected = None
        self.probability = None
        self.centroids = None
        self.cone_direction = None
        self.occupancy = None

    def _clear_parachute_result(self):
        self.parachute_detected = None
        self.parachute_centroids = None
        self.parachute_direction = None
