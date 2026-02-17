# encoding : utf-8
import cv2
import numpy as np
from picamera2 import Picamera2  # camera


class detector:
    # コンストラクタ
    # 引数
    #   roi : 対象領域
    def __init__(self):
        # 諸変数のイニシャライズ
        self.cone_ratio = 33 / 70  # コーンの縦横比
        self.ratio_thresh = 0.1  # 許容される誤差率

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
        self.picam2 = None  # camera obj.

    def set_roi_img(self):
        # ROI(透過PNG/RGBA)から、alphaマスクでコーン画素だけを使ってHSヒストを作り、混合する
        rois = ["./libs/roi1.png", "./libs/roi2.png", "./libs/roi3.png"]
        weights = [1.0, 1.0, 1.0]  # normal, far, near(noisy) など。必要なら調整

        alpha_thresh = 200  # 縁のアンチエイリアス混色を避けるなら高めが安定
        h_bins, s_bins = 180, 256  # いまの実装に合わせる（重いなら下のNOTE参照）

        mixed = np.zeros((h_bins, s_bins), np.float32)

        for path, w in zip(rois, weights):
            rgba = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if rgba is None:
                raise FileNotFoundError(path)
            if rgba.ndim != 3 or rgba.shape[2] != 4:
                raise ValueError(f"ROI must be RGBA (HxWx4). got: shape={rgba.shape}")

            # OpenCVのimreadはPNGをBGRAで読むので、BGRAとして扱うのが安全
            bgr = cv2.cvtColor(rgba, cv2.COLOR_BGRA2BGR)
            alpha = rgba[:, :, 3]

            mask = (alpha >= alpha_thresh).astype(np.uint8) * 255

            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

            hist = cv2.calcHist(
                images=[hsv],
                channels=[0, 1],               # H,S
                mask=mask,
                histSize=[h_bins, s_bins],
                ranges=[0, 180, 0, 256],
            ).astype(np.float32)

            # 確率化（sum=1）してから混合（これやるとROIの面積差に引っ張られにくい）
            s = float(hist.sum())
            if s > 0:
                hist /= s
                mixed += float(w) * hist

        # 混合確率の正規化
        mixed_sum = float(mixed.sum())
        if mixed_sum <= 0:
            raise RuntimeError("Mixed histogram is empty. Check alpha masks / threshold.")
        mixed /= mixed_sum

        # back projection用にスケールを整える（0..255）
        self.__roi_hist = mixed
        cv2.normalize(self.__roi_hist, self.__roi_hist, 0, 255, cv2.NORM_MINMAX)



    # コーンの縦横比 (横/縦) を設定
    def set_cone_ratio(self, ratio):
        self.cone_ratio = ratio

    # get camera img
    def __get_camera_img(self):
        if self.picam2 is None:
            self.picam2 = Picamera2()
            cam_conf = self.picam2.create_preview_configuration()
            self.picam2.configure(cam_conf)
            self.picam2.start()
        self.input_img = cv2.blur(self.picam2.capture_array(), (8, 8))

    # 検出
    def detect_cone(self):
        self.__get_camera_img()
        self.__back_projection()
        self.__binarization()
        self.__find_cone_centroid()

    # 逆投影法を用いて, 興味領域のヒストグラムにマッチする領域を抽出
    def __back_projection(self):
        img_hsv = cv2.cvtColor(self.input_img, cv2.COLOR_BGR2HSV)
        cv2.normalize(self.__roi_hist, self.__roi_hist, 0, 255, cv2.NORM_MINMAX)
        self.projected_img = cv2.calcBackProject(
            [img_hsv], [0, 1], self.__roi_hist, [0, 180, 0, 256], 1
        )

    # 二値化・モルフォロジー変換 (クロージング)
    # gray : 入力画像 (グレースケール)
    def __binarization(self):
        ret, th = cv2.threshold(
            self.projected_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )  # 大津の二値化
        self.binarized_img = cv2.morphologyEx(
            th, cv2.MORPH_DILATE, cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        )  # モルフォロジー変換

    # ラベリング処理によって, 特定の比の長方形 (i.e. カラーコーン) を探し, その重心と確からしさを返す
    # 確からしさ abs(長方形の縦横比 - コーンの縦横比) でとりあえず定義. 小さいほど良い
    def __find_cone_centroid(self):
        imgSize = len(self.binarized_img) * len(self.binarized_img[0])
        nlabels, labels_img, stats, centroids = cv2.connectedComponentsWithStats(
            self.binarized_img.astype(np.uint8)
        )  # バウンディングボックス取得
        if nlabels == 1:
            self.is_detected = False
            self.is_reached = False
            return

        labels_img = labels_img[1:, :]
        stats = stats[1:, :]
        centroids = centroids[1:, :]

        probabilities = np.abs(
            stats[:, cv2.CC_STAT_WIDTH] / stats[:, cv2.CC_STAT_HEIGHT] - self.cone_ratio
        )  # 値が0に近いほどコーンらしい形状 (>=0)
        self.is_detected = False
        idx_cone = -1  # コーンの要素番号

        occupacies = stats[:, cv2.CC_STAT_AREA] / imgSize

        idx_cone = (
            np.argmax(occupacies) if np.max(occupacies) > 0.001 else -1
        )  # 0.001よりも大きい画像が対象

        self.is_detected = idx_cone >= 0

        if np.max(occupacies) > 0.15:
            self.is_reached = True
            self.picam2.capture_file("./log/capture_img.png")
        else:
            self.is_reached = False

        # # 検出された領域をそれぞれ検討 (先頭は背景全体なのでパス)
        # for idx in range(1, nlabels):
        #     if (
        #         stats[idx, cv2.CC_STAT_AREA] < imgSize / 20000
        #     ):  # 極度に面積が小さいものはノイズと見做し不正 (閾値は要調整)
        #         probabilities[idx] = error_val
        #         continue
        #     self.is_detected = True
        #     if (
        #         stats[idx, cv2.CC_STAT_AREA] > imgSize / 5
        #     ):  # 入力画像のうち十分な領域を占めるなら
        #         idx_cone = idx
        #         self.is_reached = True
        #         self.picam2.capture_file("./log/capture_img.png")
        # if not idx_cone > 0:  # もし見つからなかったら
        #     self.is_reached = False
        #     idx_cone = np.argmin(probabilities)  # 最も形の領域を探す

        # 見つかったコーンの諸情報を入力
        self.occupancy = occupacies[idx_cone]
        self.detected = stats[idx_cone, :]
        self.centroids = centroids[idx_cone]
        self.probability = probabilities[idx_cone]
        self.cone_direction = (
            self.centroids[0] / self.binarized_img.shape[1]
        )  # right : 1, left : 0な領域を占めるなら
                idx_cone = idx
                self.is_reached = True
                self.picam2.capture_file("./log/capture_img.png")
        if not idx_cone > 0:  # もし見つからなかったら
            self.is_reached = False
            idx_cone = np.argmin(probabilities)  # 最も形の領域を探す
        # 見つかったコーンの諸情報を入力
        self.detected = stats[idx_cone, :]
        self.centroids = centroids[idx_cone]
        self.probability = probabilities[idx_cone]
        self.cone_direction = (
            self.centroids[0] / self.binarized_img.shape[1]
        )  # right : 1, left : 0
